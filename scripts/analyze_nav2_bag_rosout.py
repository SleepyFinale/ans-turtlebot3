#!/usr/bin/env python3
"""Extract likely Nav2/controller clues from /rosout in a time window."""

import argparse
from typing import List, Tuple

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


KEYWORDS = [
    "progress",
    "stuck",
    "failed",
    "failure",
    "collision",
    "obstacle",
    "costmap",
    "trajectory",
    "dwb",
    "controller",
    "planner",
    "transform",
    "timeout",
    "lethal",
]


def _match(msg: str) -> bool:
    text = msg.lower()
    return any(k in text for k in KEYWORDS)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_path", help="Bag directory path")
    ap.add_argument("--start", type=float, required=True, help="Start unix time (s)")
    ap.add_argument("--end", type=float, required=True, help="End unix time (s)")
    ap.add_argument(
        "--node-filter",
        default="controller_server,planner_server,bt_navigator,behavior_server,recoveries_server,velocity_smoother,local_costmap,global_costmap",
        help="Comma-separated node names to include",
    )
    ap.add_argument("--show-all", action="store_true", help="Show all matching-node logs, not just keyword hits")
    args = ap.parse_args()

    if args.end <= args.start:
        raise SystemExit("--end must be greater than --start")

    node_allow = {n.strip() for n in args.node_filter.split(",") if n.strip()}
    msg_cls = get_message("rcl_interfaces/msg/Log")

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.bag_path, storage_id="sqlite3"),
        ConverterOptions("", ""),
    )

    rows: List[Tuple[float, str, str, int]] = []
    found_rosout = False

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic != "/rosout":
            continue
        found_rosout = True
        t = t_ns / 1e9
        if t < args.start or t > args.end:
            continue
        m = deserialize_message(data, msg_cls)
        node_name = str(m.name)
        if node_allow and node_name not in node_allow:
            continue
        text = str(m.msg)
        if args.show_all or _match(text):
            rows.append((t, node_name, text, int(m.level)))

    print(f"bag: {args.bag_path}")
    print(f"window: {args.start:.3f} -> {args.end:.3f}")
    if not found_rosout:
        print("rosout: not present in bag")
        return
    print(f"matching_rosout_rows: {len(rows)}")
    for t, node, text, lvl in rows:
        print(f"{t:.3f} [{node}] level={lvl}: {text}")


if __name__ == "__main__":
    main()
