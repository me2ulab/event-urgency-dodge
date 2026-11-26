"""Foveal looming avoidance: event counts -> detection (Eqs. 13-21) -> track/tau/gate (Eqs. 22-31)
-> sidestep command (Eqs. 32-35).

Inputs per tick (matched by stamp): /ecsa/events/count, /ecsa/uav/pose, /ecsa/ball/pose (the ball pose is
read only for the Eq. 35 'ball behind' test, as printed in the paper).
Outputs: /ecsa/cmd_vel (TwistStamped, world frame), /ecsa/avoidance/status (JSON),
/ecsa/avoidance/debug_image (rgb8: fired pixels, centre crop yellow, components amber, b* green).
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .common import LOCKSTEP_QOS, from_image, tick_of, tick_time, to_image

from ecsa_core.loop import Avoider  # noqa: E402
from ecsa_core.perception import roi_bounds  # noqa: E402
from experiments.run import experiments  # noqa: E402


def _f(x):
    return None if x is None or (isinstance(x, float) and not math.isfinite(x)) else float(x)


class AvoidanceNode(Node):
    def __init__(self):
        super().__init__("ecsa_avoidance")
        self.declare_parameter("experiment", "closed_loop_ttc_check")
        _, self.P = experiments()[self.get_parameter("experiment").value]
        self.av = Avoider(self.P)
        self.buf = {}
        self.last_k = -1
        self.pub_cmd = self.create_publisher(TwistStamped, "/ecsa/cmd_vel", LOCKSTEP_QOS)
        self.pub_st = self.create_publisher(String, "/ecsa/avoidance/status", LOCKSTEP_QOS)
        self.pub_dbg = self.create_publisher(Image, "/ecsa/avoidance/debug_image", 10)
        self.create_subscription(Image, "/ecsa/events/count", lambda m: self.put(m, "counts"), LOCKSTEP_QOS)
        self.create_subscription(PoseStamped, "/ecsa/uav/pose", lambda m: self.put(m, "uav"), LOCKSTEP_QOS)
        self.create_subscription(PoseStamped, "/ecsa/ball/pose", lambda m: self.put(m, "ball"), LOCKSTEP_QOS)

    def put(self, m, key):
        k = tick_of(m.header.stamp)
        self.buf.setdefault(k, {})[key] = m
        b = self.buf[k]
        if len(b) == 3:
            del self.buf[k]
            self.step(k, b)

    def step(self, k, b):
        if k <= self.last_k:
            self.av = Avoider(self.P)        # sim restarted
        self.last_k = k
        t = tick_time(k)
        counts = from_image(b["counts"])
        p = b["uav"].pose.position
        pos = np.array([p.x, p.y, p.z])
        bx = b["ball"].pose.position.x
        out = self.av.step(counts, t, pos, None if math.isnan(bx) else bx)
        cmd = TwistStamped()
        cmd.header.stamp, cmd.header.frame_id = b["counts"].header.stamp, "world"
        cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z = map(float, out.cmd.v)
        self.pub_cmd.publish(cmd)
        g, det, tr = out.gate, out.det, self.av.tracker
        st = dict(k=k, t=t, phase=out.cmd.phase, d=_f(g.d), tau=_f(g.tau), closing=_f(g.closing),
                  looming=bool(g.looming), gate=bool(g.passed), H=tr.H, committed=tr.committed,
                  commit=tr.commit_info if (tr.commit_info and abs(tr.commit_info["t"] - t) < 1e-9) else None,
                  goal_t=self.av.ctrl.goal_t, t_cmd_ms=out.t_cmd_ms)
        self.pub_st.publish(String(data=json.dumps(st)))
        self.pub_dbg.publish(to_image(self.debug_image(counts, det), "rgb8", b["counts"].header.stamp))

    def debug_image(self, counts, det):
        img = np.zeros(counts.shape + (3,), np.uint8) + 12
        img[counts > 0] = (110, 110, 110)
        def box(b, col):
            u0, v0, u1, v1 = b
            img[v0, u0:u1 + 1] = col; img[v1, u0:u1 + 1] = col
            img[v0:v1 + 1, u0] = col; img[v0:v1 + 1, u1] = col
        cam = self.P.camera
        ul, uh, vl, vh = roi_bounds(cam, self.P.perception.roi_radius)
        for c in det.components:
            box(c.bbox, (240, 200, 60))
        if det.selected is not None:
            box(det.selected.bbox, (80, 220, 120))
        box((ul - 1, vl - 1, uh + 1, vh + 1), (242, 201, 76))
        return img


def main():
    rclpy.init()
    n = AvoidanceNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
