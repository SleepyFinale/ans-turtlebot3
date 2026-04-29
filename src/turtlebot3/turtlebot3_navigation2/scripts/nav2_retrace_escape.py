#!/usr/bin/env python3
"""Robot-side retrace escape helper for Nav2 lethal/stall recovery.

This node watches map-frame motion and active NavigateToPose status, and when a
robot appears stuck (or explicitly enters lethal space) it cancels the active
goal and sends a short retreat goal chosen from recent pose history.
"""

import math
from collections import deque
from typing import Deque, Optional, Tuple

import rclpy
from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool
import tf2_ros


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    return q


class Nav2RetraceEscape(Node):
    def __init__(self):
        super().__init__('nav2_retrace_escape')

        # Triggers and retreat-shape parameters. These let the robot respond to
        # both hard lethal-space events and slower "goal is active but the robot
        # is not making progress" stalls.
        self.declare_parameter('robot_name', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('nav2_status_topic', 'navigate_to_pose/_action/status')
        self.declare_parameter('nav2_cancel_service', 'navigate_to_pose/_action/cancel_goal')
        self.declare_parameter('lethal_topic', 'nav2_lethal_inflation')
        self.declare_parameter('collision_ahead_topic', 'nav2_collision_ahead')
        self.declare_parameter('retrace_active_topic', 'nav2_retrace_active')
        self.declare_parameter('pose_buffer_duration_sec', 20.0)
        self.declare_parameter('pose_sample_min_dist_m', 0.03)
        self.declare_parameter('pose_sample_period_sec', 0.10)
        self.declare_parameter('trigger_on_lethal', True)
        self.declare_parameter('trigger_on_stall', True)
        self.declare_parameter('stall_timeout_sec', 10.0)
        self.declare_parameter('stall_min_goal_age_sec', 12.0)
        self.declare_parameter('stall_min_history_distance_m', 0.6)
        self.declare_parameter('min_retreat_distance_m', 0.50)
        self.declare_parameter('max_retreat_distance_m', 2.0)
        self.declare_parameter('cooldown_sec', 5.0)
        self.declare_parameter('insufficient_history_warn_throttle_sec', 2.0)
        self.declare_parameter('publish_hz', 4.0)
        self.declare_parameter('tick_hz', 8.0)
        self.declare_parameter('tf_lookup_timeout_sec', 0.12)
        self.declare_parameter('retreat_result_timeout_sec', 12.0)
        self.declare_parameter('enable_collision_retry_guard', True)
        self.declare_parameter('collision_retry_window_sec', 8.0)
        self.declare_parameter('collision_retry_min_events', 5)
        self.declare_parameter('collision_retry_min_goal_age_sec', 6.0)
        self.declare_parameter('collision_retry_cooldown_sec', 10.0)
        self.declare_parameter('retry_zone_radius_m', 0.45)
        self.declare_parameter('retry_zone_decay_sec', 22.0)
        self.declare_parameter('retry_zone_hard_block_sec', 6.0)
        self.declare_parameter('retry_zone_extra_events_max', 5)
        self.declare_parameter('retry_zone_repeated_hits_min', 2)
        self.declare_parameter('retry_zone_nonrepeated_extra_events_max', 1)

        self.robot_name = str(self.get_parameter('robot_name').value or '')
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.nav2_status_topic = str(self.get_parameter('nav2_status_topic').value)
        self.nav2_cancel_service = str(self.get_parameter('nav2_cancel_service').value)
        self.lethal_topic = str(self.get_parameter('lethal_topic').value)
        self.collision_ahead_topic = str(self.get_parameter('collision_ahead_topic').value)
        self.retrace_active_topic = str(self.get_parameter('retrace_active_topic').value)
        self.pose_buffer_duration_sec = float(
            self.get_parameter('pose_buffer_duration_sec').value
        )
        self.pose_sample_min_dist_m = float(
            self.get_parameter('pose_sample_min_dist_m').value
        )
        self.pose_sample_period_sec = float(
            self.get_parameter('pose_sample_period_sec').value
        )
        self.trigger_on_lethal = bool(self.get_parameter('trigger_on_lethal').value)
        self.trigger_on_stall = bool(self.get_parameter('trigger_on_stall').value)
        self.stall_timeout_sec = float(self.get_parameter('stall_timeout_sec').value)
        self.stall_min_goal_age_sec = float(
            self.get_parameter('stall_min_goal_age_sec').value
        )
        self.stall_min_history_distance_m = float(
            self.get_parameter('stall_min_history_distance_m').value
        )
        self.min_retreat_distance_m = float(
            self.get_parameter('min_retreat_distance_m').value
        )
        self.max_retreat_distance_m = float(
            self.get_parameter('max_retreat_distance_m').value
        )
        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)
        self.insufficient_history_warn_throttle_sec = float(
            self.get_parameter('insufficient_history_warn_throttle_sec').value
        )
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.tick_hz = float(self.get_parameter('tick_hz').value)
        self.tf_lookup_timeout_sec = float(
            self.get_parameter('tf_lookup_timeout_sec').value
        )
        self.retreat_result_timeout_sec = float(
            self.get_parameter('retreat_result_timeout_sec').value
        )
        self.enable_collision_retry_guard = bool(
            self.get_parameter('enable_collision_retry_guard').value
        )
        self.collision_retry_window_sec = max(
            1.0, float(self.get_parameter('collision_retry_window_sec').value)
        )
        self.collision_retry_min_events = max(
            2, int(self.get_parameter('collision_retry_min_events').value)
        )
        self.collision_retry_min_goal_age_sec = max(
            0.0, float(self.get_parameter('collision_retry_min_goal_age_sec').value)
        )
        self.collision_retry_cooldown_sec = max(
            0.0, float(self.get_parameter('collision_retry_cooldown_sec').value)
        )
        self.retry_zone_radius_m = max(
            0.05, float(self.get_parameter('retry_zone_radius_m').value)
        )
        self.retry_zone_decay_sec = max(
            1.0, float(self.get_parameter('retry_zone_decay_sec').value)
        )
        self.retry_zone_hard_block_sec = max(
            0.0, float(self.get_parameter('retry_zone_hard_block_sec').value)
        )
        self.retry_zone_extra_events_max = max(
            0, int(self.get_parameter('retry_zone_extra_events_max').value)
        )
        self.retry_zone_repeated_hits_min = max(
            1, int(self.get_parameter('retry_zone_repeated_hits_min').value)
        )
        self.retry_zone_nonrepeated_extra_events_max = max(
            0, int(self.get_parameter('retry_zone_nonrepeated_extra_events_max').value)
        )

        # This node combines action status, TF, and costmap-derived Bool topics
        # because no single Nav2 server has enough context to choose a retreat.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cancel_client = self.create_client(CancelGoal, self.nav2_cancel_service)
        self.status_sub = self.create_subscription(
            GoalStatusArray, self.nav2_status_topic, self._status_callback, 10
        )
        self.lethal_sub = self.create_subscription(
            Bool, self.lethal_topic, self._lethal_callback, 10
        )
        self.collision_ahead_sub = self.create_subscription(
            Bool, self.collision_ahead_topic, self._collision_ahead_callback, 10
        )
        self.active_pub = self.create_publisher(Bool, self.retrace_active_topic, 10)

        # pose_buffer stores recent map-frame samples as
        # (timestamp, x, y, yaw). Retreat goals are selected from this history.
        self.pose_buffer: Deque[Tuple[float, float, float, float]] = deque()
        self.last_sample_pose: Optional[Tuple[float, float, float]] = None
        self.last_motion_reference_pose: Optional[Tuple[float, float, float]] = None
        self.last_motion_time: float = 0.0
        self.goal_start_time: float = 0.0
        self.last_active_goal_uuid: Optional[Tuple[int, ...]] = None
        self.last_insufficient_history_warn_time: float = 0.0
        self.last_active_goal_info: Optional[GoalInfo] = None
        self.goal_active: bool = False
        self.lethal_active: bool = False
        self.retrace_active: bool = False
        self.current_retreat_goal_active: bool = False
        self.retrace_last_start_time: float = 0.0
        self.retrace_last_end_time: float = 0.0
        self.pending_trigger_reason: str = ''
        self.collision_ahead_active: bool = False
        self.collision_ahead_events: Deque[float] = deque()
        self.last_collision_retry_trigger_time: float = 0.0
        self.retry_zones: Deque[dict] = deque()
        self.last_collision_pose: Optional[Tuple[float, float, float]] = None

        tick_period = 1.0 / self.tick_hz if self.tick_hz > 0.0 else 0.125
        pub_period = 1.0 / self.publish_hz if self.publish_hz > 0.0 else 0.25
        self.tick_timer = self.create_timer(tick_period, self._tick)
        self.pub_timer = self.create_timer(pub_period, self._publish_retrace_active)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _status_callback(self, msg: GoalStatusArray) -> None:
        active = None
        for st in msg.status_list:
            if st.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                active = st.goal_info
                break
        now = self._now_sec()
        active_uuid = tuple(active.goal_id.uuid) if active is not None else None
        if active_uuid is not None and active_uuid != self.last_active_goal_uuid:
            # New NavigateToPose leg: restart stall timers so we don't fire from
            # pre-goal idle time.
            self.goal_start_time = now
            self.last_motion_time = now
            self.last_insufficient_history_warn_time = 0.0
            self.collision_ahead_events.clear()
            self.collision_ahead_active = False
            if self.last_sample_pose is not None:
                self.last_motion_reference_pose = self.last_sample_pose
        if active_uuid is None:
            self.goal_start_time = 0.0
            self.last_motion_reference_pose = self.last_sample_pose
            self.collision_ahead_events.clear()
            self.collision_ahead_active = False
        self.goal_active = active is not None
        self.last_active_goal_uuid = active_uuid
        self.last_active_goal_info = active

    def _lethal_callback(self, msg: Bool) -> None:
        self.lethal_active = bool(msg.data)

    def _collision_ahead_callback(self, msg: Bool) -> None:
        now = self._now_sec()
        active = bool(msg.data)
        if active and not self.collision_ahead_active:
            # Count only rising edges so one sustained controller warning does
            # not look like repeated retries in the same blocked spot.
            self.collision_ahead_events.append(now)
            pose = self._lookup_pose()
            if pose is not None:
                self.last_collision_pose = pose
        self.collision_ahead_active = active

    def _zone_dist(self, pose: Tuple[float, float, float], zone: dict) -> float:
        px, py, _ = pose
        return math.hypot(px - zone['x'], py - zone['y'])

    def _zone_strength(self, now: float, zone: dict) -> float:
        age = max(0.0, now - float(zone['created_s']))
        if age <= self.retry_zone_hard_block_sec:
            return 1.0
        fade_age = age - self.retry_zone_hard_block_sec
        if fade_age >= self.retry_zone_decay_sec:
            return 0.0
        return max(0.0, 1.0 - (fade_age / self.retry_zone_decay_sec))

    def _prune_retry_zones(self, now: float) -> None:
        kept = deque()
        for zone in self.retry_zones:
            if self._zone_strength(now, zone) > 0.0:
                kept.append(zone)
        self.retry_zones = kept

    def _find_nearest_zone(
        self, pose: Tuple[float, float, float], now: float
    ) -> Optional[dict]:
        nearest = None
        best_dist = float('inf')
        for zone in self.retry_zones:
            dist = self._zone_dist(pose, zone)
            if dist <= self.retry_zone_radius_m and dist < best_dist:
                nearest = zone
                best_dist = dist
        if nearest is None:
            return None
        if self._zone_strength(now, nearest) <= 0.0:
            return None
        return nearest

    def _update_retry_zone(self, pose: Tuple[float, float, float], now: float) -> None:
        zone = self._find_nearest_zone(pose, now)
        px, py, _ = pose
        if zone is None:
            self.retry_zones.append({
                'x': px,
                'y': py,
                'created_s': now,
                'last_hit_s': now,
                'hits': 1,
            })
            return
        zone['last_hit_s'] = now
        zone['hits'] = int(zone.get('hits', 0)) + 1

    def _publish_retrace_active(self) -> None:
        out = Bool()
        out.data = bool(self.retrace_active)
        self.active_pub.publish(out)

    def _lookup_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None
        x = float(t.transform.translation.x)
        y = float(t.transform.translation.y)
        q = t.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return x, y, yaw

    def _update_pose_history(self, now: float, pose: Tuple[float, float, float]) -> None:
        x, y, yaw = pose
        if self.last_sample_pose is None:
            self.pose_buffer.append((x, y, yaw, now))
            self.last_sample_pose = pose
            self.last_motion_reference_pose = pose
            self.last_motion_time = now
            return

        lx, ly, _ = self.last_sample_pose
        moved = math.hypot(x - lx, y - ly)
        age = now - self.pose_buffer[-1][3] if self.pose_buffer else 0.0
        if moved >= self.pose_sample_min_dist_m or age >= self.pose_sample_period_sec:
            self.pose_buffer.append((x, y, yaw, now))
            self.last_sample_pose = pose

        if self.last_motion_reference_pose is None:
            self.last_motion_reference_pose = pose
        mx, my, _ = self.last_motion_reference_pose
        moved_since_motion_mark = math.hypot(x - mx, y - my)
        if moved_since_motion_mark >= self.pose_sample_min_dist_m:
            self.last_motion_time = now
            self.last_motion_reference_pose = pose

        while self.pose_buffer and (now - self.pose_buffer[0][3]) > self.pose_buffer_duration_sec:
            self.pose_buffer.popleft()

    def _history_span_from_current(self, current_pose: Tuple[float, float, float]) -> float:
        if not self.pose_buffer:
            return 0.0
        cx, cy, _ = current_pose
        cumulative = 0.0
        prev_x, prev_y = cx, cy
        for x, y, _yaw, _t in reversed(self.pose_buffer):
            cumulative += math.hypot(prev_x - x, prev_y - y)
            prev_x, prev_y = x, y
        return cumulative

    def _choose_retreat_target(
        self, current_pose: Tuple[float, float, float]
    ) -> Optional[Tuple[float, float, float]]:
        if len(self.pose_buffer) < 2:
            return None
        cx, cy, _ = current_pose
        cumulative = 0.0
        prev_x, prev_y = cx, cy
        for x, y, yaw, _ in reversed(self.pose_buffer):
            step = math.hypot(prev_x - x, prev_y - y)
            cumulative += step
            if cumulative >= self.max_retreat_distance_m:
                return x, y, yaw
            if cumulative >= self.min_retreat_distance_m:
                return x, y, yaw
            prev_x, prev_y = x, y
        return None

    def _cancel_active_goal_and_retrace(
        self, reason: str, retreat_pose: Tuple[float, float, float]
    ) -> None:
        if self.retrace_active:
            return
        self.retrace_active = True
        self.pending_trigger_reason = reason
        self.retrace_last_start_time = self._now_sec()
        self.current_retreat_goal_active = False

        if (
            self.last_active_goal_info is not None
            and self.cancel_client.wait_for_service(timeout_sec=0.25)
        ):
            req = CancelGoal.Request()
            req.goal_info = self.last_active_goal_info
            self.cancel_client.call_async(req)

        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Retrace requested but NavigateToPose action server unavailable')
            self.retrace_active = False
            self.retrace_last_end_time = self._now_sec()
            return

        gx, gy, gyaw = retreat_pose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = _yaw_to_quaternion(gyaw)
        self.current_retreat_goal_active = True
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_retreat_goal_response)
        self.get_logger().warn(
            f'Retrace escape triggered ({reason}); retreat goal=({gx:.2f}, {gy:.2f})'
        )

    def _on_retreat_goal_response(self, future) -> None:
        try:
            gh = future.result()
        except Exception:
            self.current_retreat_goal_active = False
            self.retrace_active = False
            self.retrace_last_end_time = self._now_sec()
            return
        if not gh.accepted:
            self.current_retreat_goal_active = False
            self.retrace_active = False
            self.retrace_last_end_time = self._now_sec()
            return
        result_future = gh.get_result_async()
        result_future.add_done_callback(self._on_retreat_result)

    def _on_retreat_result(self, _future) -> None:
        self.current_retreat_goal_active = False
        self.retrace_active = False
        self.retrace_last_end_time = self._now_sec()

    def _should_trigger(self, now: float) -> Optional[str]:
        if self.retrace_active or self.current_retreat_goal_active:
            return None
        if not self.goal_active:
            return None
        if self.retrace_last_end_time > 0.0 and (now - self.retrace_last_end_time) < self.cooldown_sec:
            return None
        if self.trigger_on_lethal and self.lethal_active:
            return 'lethal_space'
        if self.enable_collision_retry_guard:
            self._prune_retry_zones(now)
            if (
                self.goal_start_time > 0.0
                and (now - self.goal_start_time) >= self.collision_retry_min_goal_age_sec
                and (
                    self.last_collision_retry_trigger_time <= 0.0
                    or (now - self.last_collision_retry_trigger_time)
                    >= self.collision_retry_cooldown_sec
                )
            ):
                while self.collision_ahead_events and (
                    now - self.collision_ahead_events[0]
                ) > self.collision_retry_window_sec:
                    self.collision_ahead_events.popleft()
                current_pose = self.last_collision_pose or self.last_sample_pose
                zone = None
                extra_required = 0
                if current_pose is not None:
                    zone = self._find_nearest_zone(current_pose, now)
                if zone is not None:
                    strength = self._zone_strength(now, zone)
                    repeated_zone = int(zone.get('hits', 0)) >= self.retry_zone_repeated_hits_min
                    max_extra = (
                        self.retry_zone_extra_events_max
                        if repeated_zone
                        else self.retry_zone_nonrepeated_extra_events_max
                    )
                    extra_required = int(round((1.0 - strength) * max_extra))
                required_events = self.collision_retry_min_events + extra_required
                if len(self.collision_ahead_events) >= required_events:
                    if current_pose is not None:
                        self._update_retry_zone(current_pose, now)
                    self.last_collision_retry_trigger_time = now
                    if zone is not None:
                        return 'collision_retry_zone'
                    return 'collision_retry_guard'
        if self.trigger_on_stall and self.last_motion_time > 0.0:
            if self.goal_start_time > 0.0 and (now - self.goal_start_time) < self.stall_min_goal_age_sec:
                return None
            if (now - self.last_motion_time) >= self.stall_timeout_sec:
                return 'movement_stall'
        return None

    def _tick(self) -> None:
        now = self._now_sec()
        pose = self._lookup_pose()
        if pose is not None:
            self._update_pose_history(now, pose)

        if self.retrace_active and self.retrace_last_start_time > 0.0:
            if (now - self.retrace_last_start_time) > self.retreat_result_timeout_sec:
                self.get_logger().warn('Retrace timed out; clearing retrace_active')
                self.retrace_active = False
                self.current_retreat_goal_active = False
                self.retrace_last_end_time = now
            return

        if pose is None:
            return

        trigger_reason = self._should_trigger(now)
        if trigger_reason is None:
            return
        target = self._choose_retreat_target(pose)
        if target is None:
            if trigger_reason == 'movement_stall':
                hist_span = self._history_span_from_current(pose)
                if hist_span < self.stall_min_history_distance_m:
                    if (
                        now - self.last_insufficient_history_warn_time
                        >= self.insufficient_history_warn_throttle_sec
                    ):
                        self.last_insufficient_history_warn_time = now
                        self.get_logger().warn(
                            f'Retrace trigger ({trigger_reason}) ignored: '
                            f'insufficient pose history (span={hist_span:.2f}m, '
                            f'required={self.stall_min_history_distance_m:.2f}m)'
                        )
                    return
            elif (
                now - self.last_insufficient_history_warn_time
                >= self.insufficient_history_warn_throttle_sec
            ):
                self.last_insufficient_history_warn_time = now
                self.get_logger().warn(
                    f'Retrace trigger ({trigger_reason}) ignored: insufficient pose history'
                )
            return
        self._cancel_active_goal_and_retrace(trigger_reason, target)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2RetraceEscape()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, RuntimeError):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
