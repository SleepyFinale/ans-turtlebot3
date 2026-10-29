#!/usr/bin/env python3
"""Score a robot motion bag: jitter, idle time, repeated contacts.

Record on the robot (small topics only; a few MB per 10 minutes):

    ros2 bag record --include-hidden-topics -o ~/turtlebot3/logs/<name> \\
      /<robot>/cmd_vel /<robot>/odom /<robot>/stuck/event /<robot>/collision/diagnosis \\
      /<robot>/navigate_to_pose/_action/status /<robot>/cmd_vel_mux/active /rosout

then, here or on the central PC (only needs rosbag2_py):

    python3 scripts/diagnostics/analyze_motion_bag.py ~/turtlebot3/logs/<name>

Reports:
- cmd_vel rate and how often the drive direction reverses within
  flip_window_sec (a reversal that fast is two publishers fighting, or a
  controller oscillating: the robot shakes instead of moving);
- time with a NavigateToPose goal active but the robot not moving
  (idle_speed_mps for idle_min_sec or longer), listed longest first;
- time with no goal at all (the explorer between goals). Action status
  topics are hidden: without --include-hidden-topics the bag has none, and
  every stretch standing still is listed instead;
- route flips: consecutive plans to the same goal whose first 2 m differ
  by more than 0.3 m (the planner switching sides around an obstacle; needs
  /<robot>/plan in the bag);
- stuck_detector events and collision_resolver verdicts, by kind.
"""

import argparse
import json
import math
from collections import Counter

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    storage = 'mcap' if path.endswith('.mcap') else 'sqlite3'
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in classes:
            classes[topic] = get_message(types[topic])
        yield topic, deserialize_message(data, classes[topic]), t_ns * 1e-9


def _prefix(pts, length=2.0):
    """The first `length` metres of a path, resampled every 0.1 m."""
    out, travelled = [pts[0]], 0.0
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        travelled += math.hypot(bx - ax, by - ay)
        if travelled >= 0.1 * len(out):
            out.append((bx, by))
        if travelled >= length:
            break
    return out


def route_flips(plans, goal_tol=0.3, flip_m=0.3):
    """(flips, comparisons) between consecutive plans to the same goal.

    A flip: some point of the new plan's first 2 m is more than flip_m from
    every point of the previous plan, i.e. the route around something changed
    side, not just its start moving with the robot.
    """
    flips = compared = 0
    for (_ta, a), (_tb, b) in zip(plans, plans[1:]):
        if math.hypot(a[-1][0] - b[-1][0], a[-1][1] - b[-1][1]) > goal_tol:
            continue  # a new goal
        compared += 1
        if max(min(math.hypot(px - qx, py - qy) for qx, qy in a) for px, py in _prefix(b)) > flip_m:
            flips += 1
    return flips, compared


def spans(flags, times, min_len):
    """[(start, end)] of runs where flags is true, at least min_len long."""
    out, start = [], None
    for f, t in zip(flags, times):
        if f and start is None:
            start = t
        elif not f and start is not None:
            if t - start >= min_len:
                out.append((start, t))
            start = None
    if start is not None and times and times[-1] - start >= min_len:
        out.append((start, times[-1]))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--flip-window-sec', type=float, default=0.5)
    ap.add_argument('--idle-speed-mps', type=float, default=0.02)
    ap.add_argument('--idle-min-sec', type=float, default=3.0)
    args = ap.parse_args()

    cmd, odom, goal_active, plans = [], [], [], []
    stuck, diag, mux = Counter(), Counter(), Counter()
    for topic, msg, t in read_bag(args.bag):
        if topic.endswith('/cmd_vel'):
            cmd.append((t, msg.linear.x, msg.angular.z))
        elif topic.endswith('/odom'):
            v = msg.twist.twist
            odom.append((t, math.hypot(v.linear.x, v.linear.y), abs(v.angular.z)))
        elif topic.endswith('navigate_to_pose/_action/status'):
            goal_active.append((t, any(s.status in (1, 2) for s in msg.status_list)))
        elif topic.endswith('stuck/event'):
            try:
                ev = json.loads(msg.data)
                stuck['+'.join(ev.get('reasons', []))] += 1
            except ValueError:
                pass
        elif topic.endswith('collision/diagnosis'):
            try:
                diag[json.loads(msg.data).get('verdict')] += 1
            except ValueError:
                pass
        elif topic.endswith('/plan'):
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            if pts:
                plans.append((t, pts))
        elif topic.endswith('cmd_vel_mux/active'):
            mux[msg.data] += 1

    if not odom:
        print('no odom in bag')
        return
    t0, t1 = odom[0][0], odom[-1][0]
    print(f'bag: {args.bag}  ({t1 - t0:.0f} s)')

    # --- cmd_vel ---------------------------------------------------------
    if cmd:
        rate = len(cmd) / max(1e-6, cmd[-1][0] - cmd[0][0])
        lin_flips = ang_flips = 0
        last_lin = last_ang = None
        for t, v, w in cmd:
            if abs(v) > 0.01:
                if last_lin and math.copysign(1, v) != last_lin[1] and t - last_lin[0] <= args.flip_window_sec:
                    lin_flips += 1
                last_lin = (t, math.copysign(1, v))
            if abs(w) > 0.05:
                if last_ang and math.copysign(1, w) != last_ang[1] and t - last_ang[0] <= args.flip_window_sec:
                    ang_flips += 1
                last_ang = (t, math.copysign(1, w))
        # More than one publisher shows as bursts above the controller's rate.
        bursts = sum(1 for a, b in zip(cmd, cmd[1:]) if b[0] - a[0] < 0.02)
        minutes = (t1 - t0) / 60.0
        print(f'cmd_vel: {len(cmd)} msgs, {rate:.1f} Hz mean, {bursts} pairs <20 ms apart')
        print(f'  drive reversals within {args.flip_window_sec:.1f} s: {lin_flips} '
              f'({lin_flips / max(minutes, 1e-6):.1f}/min); turn reversals: {ang_flips} '
              f'({ang_flips / max(minutes, 1e-6):.1f}/min)')

    # --- how the time was spent (measured motion, odom) --------------------
    kinds = Counter()
    dist = 0.0
    for (ta, va, wa), (tb, _vb, _wb) in zip(odom, odom[1:]):
        dt = min(tb - ta, 0.5)
        dist += va * dt
        if va >= 0.18:
            kinds['full speed (>=0.18 m/s)'] += dt
        elif va >= 0.05:
            kinds['slowed (0.05-0.18 m/s)'] += dt
        elif wa >= 0.2:
            kinds['turning in place'] += dt
        else:
            kinds['stopped'] += dt
    total = sum(kinds.values())
    print(f'distance driven {dist:.1f} m, mean {dist / max(total, 1e-6):.2f} m/s; time: ' + ', '.join(
        f'{k} {v:.0f} s ({100 * v / max(total, 1e-6):.0f}%)' for k, v in kinds.most_common()))

    # --- idle ------------------------------------------------------------
    times = [o[0] for o in odom]
    moving = [o[1] > args.idle_speed_mps or o[2] > 0.1 for o in odom]
    active = []
    j, state = 0, False
    for t in times:
        while j < len(goal_active) and goal_active[j][0] <= t:
            state = goal_active[j][1]
            j += 1
        active.append(state)
    if not goal_active:
        still = spans([not m for m in moving], times, args.idle_min_sec)
        total = sum(b - a for a, b in still)
        print(f'no NavigateToPose status in the bag (record with --include-hidden-topics); '
              f'standing still (>= {args.idle_min_sec:.0f} s): {total:.0f} s in {len(still)} spans '
              f'({100 * total / max(t1 - t0, 1e-6):.0f}%)')
        for a, b in sorted(still, key=lambda s: s[0] - s[1])[:8]:
            print(f'  {b - a:5.1f} s at t+{a - t0:.0f}')
        active = []
    idle_goal = spans([a and not m for a, m in zip(active, moving)], times, args.idle_min_sec)
    no_goal = spans([not a for a in active], times, args.idle_min_sec) if active else []
    total_goal = sum(b - a for a, b in spans(active, times, 0.0))
    idle_sum = sum(b - a for a, b in idle_goal)
    if goal_active:
        print(f'goal active {total_goal:.0f} s; standing still with a goal (>= {args.idle_min_sec:.0f} s): '
              f'{idle_sum:.0f} s in {len(idle_goal)} spans ({100 * idle_sum / max(total_goal, 1e-6):.0f}%)')
    for a, b in sorted(idle_goal, key=lambda s: s[0] - s[1])[:8]:
        print(f'  {b - a:5.1f} s at t+{a - t0:.0f}')
    nog = sum(b - a for a, b in no_goal)
    if goal_active:
        print(f'no goal (explorer between goals, >= {args.idle_min_sec:.0f} s): {nog:.0f} s in {len(no_goal)} spans')
    for a, b in sorted(no_goal, key=lambda s: s[0] - s[1])[:5]:
        print(f'  {b - a:5.1f} s at t+{a - t0:.0f}')

    if plans:
        flips, compared = route_flips(plans)
        minutes = (t1 - t0) / 60.0
        print(f'plans: {len(plans)}; route flips (first 2 m moved > 0.3 m from the previous plan, '
              f'same goal): {flips} of {compared} ({flips / max(minutes, 1e-6):.1f}/min)')

    if stuck:
        print('stuck_detector events: ' + ', '.join(f'{k}={v}' for k, v in stuck.most_common()))
    if diag:
        print('collision_resolver verdicts: ' + ', '.join(f'{k}={v}' for k, v in diag.most_common()))
    if mux:
        print('cmd_vel_mux source changes: ' + ', '.join(f'{k}={v}' for k, v in mux.most_common()))


if __name__ == '__main__':
    main()
