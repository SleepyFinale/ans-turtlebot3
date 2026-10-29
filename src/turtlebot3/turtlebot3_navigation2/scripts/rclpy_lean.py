"""Cheaper rclpy on the Pi: fewer waitables per wake-up, raw action status.

rclpy's single-threaded executor rebuilds its wait set in Python on every
wake-up, walking every subscription, timer, client and event handler of the
node. Profiling stuck_detector on Inky (about 90 wake-ups a second) put
almost half of its CPU there. Two things here cut it down:

- lean_events(node): every publisher and subscription rclpy creates gets a
  default "incompatible QoS" event handler, a waitable whose only job is a
  warning in the log. Called right after Node.__init__, before anything is
  created, it makes the node's create_publisher / create_subscription (and
  those of a TransformListener built on it) skip that handler. Nothing in
  this package uses the events.

- goal_statuses(raw): Nav2's action servers keep every finished goal for 15
  minutes and publish the whole list on every change, so an
  .../_action/status message held over 200 goals after a few minutes of
  exploring, each turned into Python objects (UUID, Time, GoalInfo) when
  deserialized. Subscribe with raw=True and read (uuid, status) pairs
  straight from the CDR bytes instead.
"""

import struct
from typing import List, Tuple

from rclpy.qos_event import PublisherEventCallbacks, SubscriptionEventCallbacks


def lean_events(node) -> None:
    create_publisher = node.create_publisher
    create_subscription = node.create_subscription

    def publisher(*args, **kwargs):
        kwargs.setdefault('event_callbacks', PublisherEventCallbacks(use_default_callbacks=False))
        return create_publisher(*args, **kwargs)

    def subscription(*args, **kwargs):
        kwargs.setdefault('event_callbacks', SubscriptionEventCallbacks(use_default_callbacks=False))
        return create_subscription(*args, **kwargs)

    node.create_publisher = publisher
    node.create_subscription = subscription


def goal_statuses(raw: bytes) -> List[Tuple[bytes, int]]:
    """(goal uuid, status) for each entry of a serialized action_msgs/GoalStatusArray.

    CDR (little-endian, 4-byte encapsulation header): a uint32 count, then per
    entry uuid uint8[16], stamp int32 sec + uint32 nanosec (each aligned to
    4 from the start of the payload), status int8.
    """
    if len(raw) < 8:
        return []
    count = struct.unpack_from('<I', raw, 4)[0]
    out = []
    pos = 8  # payload offset 4 (after the count), plus the 4-byte header
    for _ in range(count):
        uuid = raw[pos:pos + 16]
        pos += 16
        pos += (-(pos - 4)) % 4      # align the stamp to 4 within the payload
        status = struct.unpack_from('<b', raw, pos + 8)[0]
        pos += 9
        out.append((uuid, status))
    return out
