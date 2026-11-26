"""Shared helpers: repository import path, exact tick stamps, image <-> numpy."""
import sys

import numpy as np
from builtin_interfaces.msg import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from ._root import ROOT

if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

# every lockstep topic: reliable, keep all of a short history (no tick may be dropped)
LOCKSTEP_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50)
LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
TICK_NS = 25_000_000                       # 40 Hz camera period, exact in nanoseconds


def tick_stamp(k: int) -> Time:
    ns = k * TICK_NS
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def stamp_ns(stamp: Time) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def tick_of(stamp: Time) -> int:
    return stamp_ns(stamp) // TICK_NS


def tick_time(k: int) -> float:
    return round(k * TICK_NS * 1e-9, 9)


def to_image(arr: np.ndarray, encoding: str, stamp: Time, frame: str = "camera") -> Image:
    m = Image()
    m.header.stamp, m.header.frame_id = stamp, frame
    m.height, m.width = arr.shape[:2]
    m.encoding = encoding
    m.is_bigendian = 0
    a = np.ascontiguousarray(arr)
    m.step = a.strides[0]
    m.data = a.tobytes()
    return m


_DT = {"mono8": (np.uint8, 1), "rgb8": (np.uint8, 3), "32SC1": (np.int32, 1)}


def from_image(m: Image) -> np.ndarray:
    dt, ch = _DT[m.encoding]
    a = np.frombuffer(bytes(m.data), dt)
    return a.reshape(m.height, m.width) if ch == 1 else a.reshape(m.height, m.width, ch)


def polarity_rgb(pol: np.ndarray) -> np.ndarray:
    """ON red, OFF blue on a near-black background (the paper's event colours)."""
    out = np.zeros(pol.shape + (3,), np.uint8) + 12
    out[pol > 0] = (234, 56, 56)
    out[pol < 0] = (51, 122, 242)
    return out
