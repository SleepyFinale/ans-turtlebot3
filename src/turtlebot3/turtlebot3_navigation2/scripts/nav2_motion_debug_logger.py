#!/usr/bin/env python3
"""
Structured Nav2 motion debug logger.

Records synchronized navigation context (goal/map/costmap/plan/velocity/action
status) into JSONL for postmortem analysis.
"""

import json
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.msg import Costmap
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros


LETHAL_COST = 254
HIGH_COST = 200
UNKNOWN_COST = 255


def _now_iso() -> str:
    return datetime.utcnow().isoformat(timespec='milliseconds') + 'Z'


def _distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class Nav2MotionDebugLogger(Node):
    def __init__(self) -> None:
        super().__init__('nav2_motion_debug_logger')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('output_dir', '~/.ros/nav2_debug')
        self.declare_parameter('log_rate_hz', 5.0)
        self.declare_parameter('base_frame_candidates', 'base_footprint,base_link')

        output_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        robot_name = self.get_parameter('robot_name').value or (
            os.environ.get('USER') or os.environ.get('LOGNAME') or 'robot'
        )
        self._rate_hz = float(self.get_parameter('log_rate_hz').value)
        self._rate_hz = max(0.5, min(self._rate_hz, 20.0))
        self._base_frame_candidates = [
            s.strip() for s in self.get_parameter('base_frame_candidates').value.split(',')
            if s.strip()
        ] or ['base_footprint', 'base_link']

        self._session_dir = Path(output_dir) / robot_name
        self._session_dir.mkdir(parents=True, exist_ok=True)
        session_name = datetime.now().strftime('session-%Y%m%d-%H%M%S.jsonl')
        self._jsonl_path = self._session_dir / session_name
        self._jsonl_file = self._jsonl_path.open('a', encoding='utf-8')

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        qos_map = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos_default = QoSProfile(depth=20)

        self._latest_map: Optional[OccupancyGrid] = None
        self._latest_costmap: Optional[Costmap] = None
        self._latest_plan: Optional[NavPath] = None
        self._latest_odom: Optional[Odometry] = None
        self._last_goal_pose: Optional[PoseStamped] = None
        self._last_cmd_vel: Optional[Twist] = None
        self._last_cmd_vel_nav: Optional[Twist] = None
        self._last_action_status: Optional[GoalStatusArray] = None
        self._last_warn_key = ''

        self.create_subscription(OccupancyGrid, 'map', self._on_map, qos_map)
        self.create_subscription(Costmap, 'global_costmap/costmap', self._on_costmap, qos_map)
        self.create_subscription(NavPath, 'plan', self._on_plan, qos_default)
        self.create_subscription(Odometry, 'odom', self._on_odom, qos_default)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, qos_default)
        self.create_subscription(Twist, 'cmd_vel_nav', self._on_cmd_vel_nav, qos_default)
        self.create_subscription(GoalStatusArray, 'navigate_to_pose/_action/status',
                                 self._on_action_status, qos_default)
        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal_pose, qos_default)

        period = 1.0 / self._rate_hz
        self.create_timer(period, self._on_tick)

        self.get_logger().info(
            f'Nav2 motion debug logger started; JSONL={self._jsonl_path}, rate={self._rate_hz:.1f}Hz'
        )
        self._write_event('session_start', {'jsonl_path': str(self._jsonl_path)})

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _on_costmap(self, msg: Costmap) -> None:
        self._latest_costmap = msg

    def _on_plan(self, msg: NavPath) -> None:
        self._latest_plan = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd_vel = msg

    def _on_cmd_vel_nav(self, msg: Twist) -> None:
        self._last_cmd_vel_nav = msg

    def _on_action_status(self, msg: GoalStatusArray) -> None:
        self._last_action_status = msg

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        self._last_goal_pose = msg

    def _write_event(self, event: str, payload: Dict) -> None:
        line = {
            'ts_iso': _now_iso(),
            'event': event,
            **payload,
        }
        self._jsonl_file.write(json.dumps(line, separators=(',', ':')) + '\n')
        self._jsonl_file.flush()

    def _lookup_robot_pose_in_map(self) -> Optional[Tuple[float, float]]:
        for frame in self._base_frame_candidates:
            try:
                t = self._tf_buffer.lookup_transform('map', frame, rclpy.time.Time())
                return (float(t.transform.translation.x), float(t.transform.translation.y))
            except Exception:
                continue
        return None

    def _cost_at(self, cm: Costmap, x: float, y: float) -> Optional[int]:
        info = cm.metadata
        res = float(info.resolution)
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        w = int(info.size_x)
        h = int(info.size_y)
        if w <= 0 or h <= 0 or res <= 0.0:
            return None
        mx = int((x - ox) / res)
        my = int((y - oy) / res)
        if mx < 0 or mx >= w or my < 0 or my >= h:
            return None
        idx = my * w + mx
        if idx < 0 or idx >= len(cm.data):
            return None
        return int(cm.data[idx])

    def _plan_metrics(self, plan: Optional[NavPath], cm: Optional[Costmap]) -> Dict:
        if plan is None or not plan.poses:
            return {
                'plan_points': 0,
                'plan_length_m': None,
                'plan_goal_x': None,
                'plan_goal_y': None,
                'plan_min_cost': None,
                'plan_max_cost': None,
                'plan_min_clearance_m': None,
                'plan_out_of_costmap_points': None,
            }

        points: List[Tuple[float, float]] = [
            (float(p.pose.position.x), float(p.pose.position.y)) for p in plan.poses
        ]
        plen = 0.0
        for i in range(1, len(points)):
            plen += _distance(points[i - 1], points[i])

        plan_goal_x = points[-1][0]
        plan_goal_y = points[-1][1]

        if cm is None:
            return {
                'plan_points': len(points),
                'plan_length_m': plen,
                'plan_goal_x': plan_goal_x,
                'plan_goal_y': plan_goal_y,
                'plan_min_cost': None,
                'plan_max_cost': None,
                'plan_min_clearance_m': None,
                'plan_out_of_costmap_points': None,
            }

        costs: List[int] = []
        out_of_bounds = 0
        for x, y in points:
            c = self._cost_at(cm, x, y)
            if c is None:
                out_of_bounds += 1
                continue
            costs.append(c)

        # Approximate min clearance using nearest non-low-cost sample along plan.
        # We compute distance to the nearest plan point with non-low cost.
        min_clearance = None
        risky_points = [points[i] for i, c in enumerate(costs) if c >= HIGH_COST]
        if risky_points:
            best = float('inf')
            for p in points:
                for rp in risky_points:
                    d = _distance(p, rp)
                    if d < best:
                        best = d
            min_clearance = best

        return {
            'plan_points': len(points),
            'plan_length_m': plen,
            'plan_goal_x': plan_goal_x,
            'plan_goal_y': plan_goal_y,
            'plan_min_cost': min(costs) if costs else None,
            'plan_max_cost': max(costs) if costs else None,
            'plan_min_clearance_m': min_clearance,
            'plan_out_of_costmap_points': out_of_bounds,
        }

    def _extract_action_status(self) -> Dict:
        if self._last_action_status is None or not self._last_action_status.status_list:
            return {'nav2_status_count': 0, 'nav2_latest_status': None}
        latest = self._last_action_status.status_list[-1]
        return {
            'nav2_status_count': len(self._last_action_status.status_list),
            'nav2_latest_status': int(latest.status),
            'nav2_latest_goal_id': ''.join(f'{b:02x}' for b in latest.goal_info.goal_id.uuid),
        }

    def _on_tick(self) -> None:
        pose_map = self._lookup_robot_pose_in_map()
        plan_metrics = self._plan_metrics(self._latest_plan, self._latest_costmap)
        action_status = self._extract_action_status()

        robot_cost = None
        if pose_map and self._latest_costmap:
            robot_cost = self._cost_at(self._latest_costmap, pose_map[0], pose_map[1])

        goal_xy = None
        if self._last_goal_pose is not None:
            goal_xy = (
                float(self._last_goal_pose.pose.position.x),
                float(self._last_goal_pose.pose.position.y),
            )
        elif plan_metrics['plan_goal_x'] is not None:
            goal_xy = (float(plan_metrics['plan_goal_x']), float(plan_metrics['plan_goal_y']))

        dist_to_goal = _distance(pose_map, goal_xy) if pose_map and goal_xy else None
        plan_goal_error = None
        if goal_xy and plan_metrics['plan_goal_x'] is not None:
            plan_goal_error = _distance(
                goal_xy, (float(plan_metrics['plan_goal_x']), float(plan_metrics['plan_goal_y']))
            )

        cmd_vel = self._last_cmd_vel
        cmd_vel_nav = self._last_cmd_vel_nav

        anomalies: List[str] = []
        if robot_cost is not None and robot_cost >= LETHAL_COST:
            anomalies.append('robot_in_lethal_cost')
        elif robot_cost is not None and robot_cost >= HIGH_COST:
            anomalies.append('robot_in_high_cost')

        if plan_metrics['plan_out_of_costmap_points'] is not None and \
                plan_metrics['plan_out_of_costmap_points'] > 0:
            anomalies.append('plan_has_out_of_global_costmap_points')

        if (plan_metrics['plan_max_cost'] is not None and plan_metrics['plan_max_cost'] < HIGH_COST
                and robot_cost is not None and robot_cost >= HIGH_COST):
            anomalies.append('robot_in_high_cost_while_plan_low_cost')

        if (cmd_vel is not None and robot_cost is not None and robot_cost >= HIGH_COST
                and abs(float(cmd_vel.linear.x)) > 0.03):
            anomalies.append('forward_cmd_in_high_cost')

        if dist_to_goal is not None and dist_to_goal < 0.1 and robot_cost is not None and \
                robot_cost >= HIGH_COST:
            anomalies.append('goal_reached_near_obstacle')

        payload = {
            'robot_pose_map_x': pose_map[0] if pose_map else None,
            'robot_pose_map_y': pose_map[1] if pose_map else None,
            'robot_cost': robot_cost,
            'goal_x': goal_xy[0] if goal_xy else None,
            'goal_y': goal_xy[1] if goal_xy else None,
            'robot_to_goal_m': dist_to_goal,
            'plan_goal_error_m': plan_goal_error,
            **plan_metrics,
            'cmd_vel': {
                'lin_x': float(cmd_vel.linear.x) if cmd_vel else None,
                'ang_z': float(cmd_vel.angular.z) if cmd_vel else None,
            },
            'cmd_vel_nav': {
                'lin_x': float(cmd_vel_nav.linear.x) if cmd_vel_nav else None,
                'ang_z': float(cmd_vel_nav.angular.z) if cmd_vel_nav else None,
            },
            'odom_twist': {
                'lin_x': float(self._latest_odom.twist.twist.linear.x) if self._latest_odom else None,
                'ang_z': float(self._latest_odom.twist.twist.angular.z) if self._latest_odom else None,
            },
            'topic_alive': {
                'map': self._latest_map is not None,
                'global_costmap': self._latest_costmap is not None,
                'plan': self._latest_plan is not None,
                'odom': self._latest_odom is not None,
                'goal_pose': self._last_goal_pose is not None,
            },
            **action_status,
            'anomalies': anomalies,
        }
        self._write_event('tick', payload)

        warn_key = ','.join(anomalies)
        if anomalies and warn_key != self._last_warn_key:
            self.get_logger().warn(
                f'Nav2 anomaly detected: {warn_key} '
                f'(robot_cost={robot_cost}, plan_max_cost={plan_metrics["plan_max_cost"]})'
            )
            self._last_warn_key = warn_key
        elif not anomalies:
            self._last_warn_key = ''

    def destroy_node(self) -> bool:
        try:
            self._write_event('session_end', {})
            self._jsonl_file.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2MotionDebugLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
