#!/usr/bin/env python3
"""Drive the robot home when it loses its link to the central PC.

Started by start_zenoh_robot.sh, so it runs only in a fleet run. Every 2 s it
pings the central PC. After loss_sec with no reply the robot is on its own:
Nav2 runs here, on the robot's own SLAM map, so this node sends it a
NavigateToPose back to where the robot stood when the node started. The new
goal replaces whatever goal the central explorer had given.

Wi-Fi comes back by itself: wpa_supplicant keeps scanning while the robot
drives home (that is how Inky reconnected after hall2, 2026-11-12). This
node logs the Wi-Fi state while the link is down, and if a sudo rule lets
the robot user run wpa_cli it also asks wpa_supplicant to reassociate.

Once the central PC has answered for restore_sec, a trip home still under way
is cancelled and the robot waits. The explorer sends it new goals as soon as
it sees the robot's TF again.

TF is read best-effort: a reliable subscription on /<robot>/tf blocks
slam_toolbox (see tf_zenoh_publish.py).

Usage: fleet_link_watchdog.py <robot> <central_host>
Set FLEET_LINK_FAILSAFE=0 to log link changes without moving the robot.
Test without touching Wi-Fi: while /tmp/fleet-link-down exists, every ping
counts as failed (touch it to simulate a lost link, remove it to restore).
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

PING_PERIOD_S = 2.0
LOSS_S = 15.0           # no reply for this long -> drive home
RESTORE_S = 6.0         # replies for this long -> hand back to the explorer
HOME_TOLERANCE_M = 0.3  # closer than this to home: do not send a goal
RETRY_S = 10.0          # wait after a failed trip home before trying again
MAX_ATTEMPTS = 5
WIFI_LOG_S = 10.0
REASSOCIATE_S = 30.0
SIMULATE_DOWN_FILE = '/tmp/fleet-link-down'

_BEST_EFFORT = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _compose(a, b):
    """(x, y, yaw) of b expressed through a: a * b."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def _wifi_state() -> str:
    try:
        out = subprocess.run(['iw', 'dev', 'wlan0', 'link'], capture_output=True,
                             text=True, timeout=3).stdout
    except (OSError, subprocess.TimeoutExpired):
        return 'unknown'
    if 'Not connected' in out:
        return 'not connected'
    ssid = signal = '?'
    for line in out.splitlines():
        line = line.strip()
        if line.startswith('SSID:'):
            ssid = line.split(':', 1)[1].strip()
        elif line.startswith('signal:'):
            signal = line.split(':', 1)[1].strip()
    return f'connected to {ssid}, signal {signal}'


class LinkProbe(threading.Thread):
    """Pings the central PC in the background; ping can block for seconds."""

    def __init__(self, host: str) -> None:
        super().__init__(daemon=True)
        self.host = host
        self.lock = threading.Lock()
        self.last_ok = 0.0
        self.ok_since = 0.0   # start of the current run of replies, 0 if the last ping failed

    def run(self) -> None:
        while True:
            started = time.monotonic()
            if os.path.exists(SIMULATE_DOWN_FILE):
                ok = False
            else:
                ok = self._ping()
            now = time.monotonic()
            with self.lock:
                if ok:
                    self.last_ok = now
                    if self.ok_since == 0.0:
                        self.ok_since = now
                else:
                    self.ok_since = 0.0
            time.sleep(max(0.0, PING_PERIOD_S - (now - started)))

    def _ping(self) -> bool:
        try:
            return subprocess.run(['ping', '-c', '1', '-W', '2', self.host],
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                  timeout=4).returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            return False

    def snapshot(self):
        with self.lock:
            return self.last_ok, self.ok_since


class FleetLinkWatchdog(Node):
    def __init__(self, robot: str, central_host: str) -> None:
        super().__init__(f'fleet_link_watchdog_{robot}')
        self.robot = robot
        self.enabled = os.environ.get('FLEET_LINK_FAILSAFE', '1') != '0'
        self.map_frame = f'{robot}/map'
        self.odom_frame = f'{robot}/odom'
        self.base_frame = f'{robot}/base_footprint'
        self.map_odom = None
        self.odom_base = None
        self.home = None
        self.state = 'waiting'     # waiting -> linked <-> lost
        self.lost_at = 0.0
        self.goal_handle = None
        self.goal_busy = False     # a goal is being sent or is under way
        self.attempts = 0
        self.next_attempt = 0.0
        self.arrived = False
        self.last_wifi_log = 0.0
        self.last_reassociate = 0.0
        self.may_reassociate = True

        self.create_subscription(TFMessage, f'/{robot}/tf', self._on_tf, _BEST_EFFORT)
        self.nav = ActionClient(self, NavigateToPose, f'/{robot}/navigate_to_pose')
        self.probe = LinkProbe(central_host)
        self.probe.start()
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'Link watchdog for {robot}: pinging {central_host}; drive home after '
            f'{LOSS_S:.0f} s without a reply'
            + ('' if self.enabled else ' (FLEET_LINK_FAILSAFE=0: logging only)'))

    # -- pose ---------------------------------------------------------------

    def _on_tf(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            parent = t.header.frame_id.lstrip('/')
            child = t.child_frame_id.lstrip('/')
            pose = (t.transform.translation.x, t.transform.translation.y,
                    _yaw(t.transform.rotation))
            if parent == self.map_frame and child == self.odom_frame:
                self.map_odom = pose
            elif parent == self.odom_frame and child == self.base_frame:
                self.odom_base = pose

    def _pose(self):
        if self.map_odom is None or self.odom_base is None:
            return None
        return _compose(self.map_odom, self.odom_base)

    # -- state machine --------------------------------------------------------

    def _tick(self) -> None:
        now = time.monotonic()
        last_ok, ok_since = self.probe.snapshot()
        pose = self._pose()

        if self.state == 'waiting':
            if last_ok > 0.0 and pose is not None:
                self.home = pose
                self.state = 'linked'
                self.get_logger().info(
                    f'Link up; home set to ({pose[0]:.2f}, {pose[1]:.2f}) in {self.map_frame}')
            return

        if self.state == 'linked':
            if now - last_ok > LOSS_S:
                self._on_lost(now, pose)
            return

        # state == 'lost'
        if ok_since > 0.0 and now - ok_since >= RESTORE_S:
            self._on_restored(now)
            return
        self._log_wifi(now)
        self._drive_home(now, pose)

    def _on_lost(self, now: float, pose) -> None:
        self.state = 'lost'
        self.lost_at = now
        self.attempts = 0
        self.next_attempt = now
        self.arrived = False
        self.last_wifi_log = 0.0
        self.get_logger().warn(
            f'No reply from the central PC for {LOSS_S:.0f} s: link lost; '
            + ('driving home and waiting for Wi-Fi' if self.enabled else 'logging only')
            + f' (Wi-Fi: {_wifi_state()})')

    def _on_restored(self, now: float) -> None:
        self.state = 'linked'
        outage = now - self.lost_at
        if self.goal_handle is not None and self.goal_busy:
            self.goal_handle.cancel_goal_async()
            what = 'cancelled the trip home; waiting for the explorer'
        elif self.arrived:
            what = 'robot is at home; waiting for the explorer'
        else:
            what = 'waiting for the explorer'
        self.goal_handle = None
        self.goal_busy = False
        self.get_logger().info(f'Link back after {outage:.0f} s; {what}')

    def _log_wifi(self, now: float) -> None:
        if now - self.last_wifi_log >= WIFI_LOG_S:
            self.last_wifi_log = now
            self.get_logger().info(
                f'Link down {now - self.lost_at:.0f} s; Wi-Fi: {_wifi_state()}')
        if self.may_reassociate and now - self.last_reassociate >= REASSOCIATE_S:
            self.last_reassociate = now
            # Needs a sudo rule for the robot user; without one wpa_supplicant
            # still reconnects on its own, so this is only a nudge.
            try:
                result = subprocess.run(
                    ['sudo', '-n', 'wpa_cli', '-i', 'wlan0', 'reassociate'],
                    capture_output=True, text=True, timeout=5)
            except (OSError, subprocess.TimeoutExpired):
                return
            if result.returncode != 0:
                self.may_reassociate = False
                self.get_logger().info(
                    'No sudo rule for wpa_cli; relying on wpa_supplicant to reconnect')

    def _drive_home(self, now: float, pose) -> None:
        if not self.enabled or self.arrived or self.goal_busy or now < self.next_attempt:
            return
        if pose is not None and math.hypot(pose[0] - self.home[0], pose[1] - self.home[1]) < HOME_TOLERANCE_M:
            self.arrived = True
            self.get_logger().info('Already at home; waiting for Wi-Fi')
            return
        if self.attempts >= MAX_ATTEMPTS:
            return
        if not self.nav.server_is_ready():
            self.next_attempt = now + 2.0
            return
        self.attempts += 1
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.home[0]
        goal.pose.pose.position.y = self.home[1]
        goal.pose.pose.orientation.z = math.sin(self.home[2] / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.home[2] / 2.0)
        self.goal_busy = True
        self.get_logger().info(
            f'Sending trip home to ({self.home[0]:.2f}, {self.home[1]:.2f}), '
            f'attempt {self.attempts} of {MAX_ATTEMPTS}')
        self.nav.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.goal_busy = False
            self.next_attempt = time.monotonic() + RETRY_S
            self.get_logger().warn('Nav2 refused the trip home; will try again')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, h=handle: self._on_goal_result(h, f))

    def _on_goal_result(self, handle, future) -> None:
        if handle is not self.goal_handle:
            return  # a trip cancelled when the link came back
        self.goal_handle = None
        self.goal_busy = False
        status = future.result().status
        if self.state != 'lost':
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.arrived = True
            self.get_logger().info('Home reached; waiting for Wi-Fi')
        elif self.attempts >= MAX_ATTEMPTS:
            self.get_logger().error(
                f'Trip home failed {MAX_ATTEMPTS} times; staying put and waiting for Wi-Fi')
        else:
            self.next_attempt = time.monotonic() + RETRY_S
            self.get_logger().warn(
                f'Trip home ended with status {status}; trying again in {RETRY_S:.0f} s')


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    rclpy.init(args=sys.argv[3:])
    node = FleetLinkWatchdog(sys.argv[1], sys.argv[2])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
