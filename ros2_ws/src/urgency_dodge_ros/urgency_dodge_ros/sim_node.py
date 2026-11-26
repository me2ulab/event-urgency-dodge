"""Lockstep plant + renderer: owns the clock, the vehicle pose (Eq. 2), the ball (Eq. 38) and scoring.

Tick k: render at the current pose -> publish image, UAV pose, ball pose, /clock (all stamped k*25 ms)
-> wait for /ecsa/cmd_vel stamped for tick k -> integrate -> tick k+1. Nothing advances without the
command, so the ROS run is deterministic and equal to the offline runner (tests/test_ros_equivalence.py).

Parameters: experiment (name from experiments/run.py), realtime (0 = as fast as possible, 1 = 40 Hz
wall clock for watching in RViz), output_dir, gz_gui.
"""
import json
import math
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .common import LATCHED_QOS, LOCKSTEP_QOS, ROOT, tick_of, tick_stamp, tick_time, to_image

from ecsa_core.control import Ball, segment_min_distance  # noqa: E402
from experiments.run import experiments, make_renderer  # noqa: E402


class SimNode(Node):
    def __init__(self):
        super().__init__("ecsa_sim")
        self.declare_parameter("experiment", "closed_loop_ttc_check")
        self.declare_parameter("realtime", 0.0)
        self.declare_parameter("output_dir", str(Path(ROOT) / "results" / "ros"))
        self.declare_parameter("start_delay", 2.0)
        self.exp = self.get_parameter("experiment").value
        self.kind, self.P = experiments()[self.exp]
        self.rt = float(self.get_parameter("realtime").value)
        self.out = Path(self.get_parameter("output_dir").value) / self.exp
        self.pub_img = self.create_publisher(__import__("sensor_msgs.msg", fromlist=["Image"]).Image,
                                             "/ecsa/camera/image_raw", LOCKSTEP_QOS)
        self.pub_uav = self.create_publisher(PoseStamped, "/ecsa/uav/pose", LOCKSTEP_QOS)
        self.pub_ball = self.create_publisher(PoseStamped, "/ecsa/ball/pose", LOCKSTEP_QOS)
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.pub_done = self.create_publisher(String, "/ecsa/sim/summary", LATCHED_QOS)
        self.pub_exp = self.create_publisher(String, "/ecsa/experiment", LATCHED_QOS)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(TwistStamped, "/ecsa/cmd_vel", self.on_cmd, LOCKSTEP_QOS)
        self.create_subscription(String, "/ecsa/avoidance/status", self.on_status, LOCKSTEP_QOS)
        self._cmds = {}
        self._status = {}
        self._cv = threading.Condition()
        self.pub_exp.publish(String(data=json.dumps(dict(name=self.exp, backend=self.kind, params=self.P.to_dict()))))
        self.get_logger().info(f"experiment {self.exp} on backend {self.kind}")

    def on_cmd(self, m: TwistStamped):
        with self._cv:
            self._cmds[tick_of(m.header.stamp)] = np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z])
            self._cv.notify_all()

    def on_status(self, m: String):
        s = json.loads(m.data)
        self._status[s["k"]] = s

    def _pose(self, xyz, stamp, frame="world"):
        p = PoseStamped()
        p.header.stamp, p.header.frame_id = stamp, frame
        p.pose.position.x, p.pose.position.y, p.pose.position.z = map(float, xyz)
        p.pose.orientation.w = 1.0
        return p

    def wait_for_graph(self):
        """Start only when the event and avoidance nodes are listening (no tick may be lost)."""
        while rclpy.ok() and (self.pub_img.get_subscription_count() == 0 or self.pub_uav.get_subscription_count() == 0
                              or self.count_publishers("/ecsa/cmd_vel") == 0):
            time.sleep(0.1)
        time.sleep(float(self.get_parameter("start_delay").value))

    def run(self):
        P = self.P
        dt = P.camera.dt
        renderer = make_renderer(self.kind, P)
        ball = Ball(P)
        c, e = P.control, P.encounter
        t_max = max(P.t_max, e.t_takeoff(c) + (c.goal_x - e.x0) / c.v_cruise + 2.0)
        pos = np.array([e.x0, 0.0, e.z_pad])
        self.wait_for_graph()
        self.get_logger().info("graph ready, starting lockstep")
        rows, sep_min, sep_min_t, hit, k = [], math.inf, None, False, 0
        wall0 = time.time()
        try:
            while rclpy.ok():
                t = tick_time(k)
                st = tick_stamp(k)
                bpos = ball.pos(t) if ball.present else None
                img = renderer.render(pos.copy(), None if bpos is None else bpos.copy(), t)
                if img.dtype != np.uint8:
                    img = (np.clip(img, 0, 1) * 255).round().astype(np.uint8)   # renderers return uint8; guard only
                self.pub_clock.publish(Clock(clock=st))
                self.pub_uav.publish(self._pose(pos, st))
                self.pub_ball.publish(self._pose(bpos if bpos is not None else (math.nan, 0, 0), st))
                tf = TransformStamped()
                tf.header.stamp, tf.header.frame_id, tf.child_frame_id = st, "world", "uav"
                tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, pos)
                tf.transform.rotation.w = 1.0
                self.tf.sendTransform(tf)
                self.pub_img.publish(to_image(img, "mono8", st))
                with self._cv:                                   # lockstep: wait for this tick's command
                    if not self._cv.wait_for(lambda: k in self._cmds, timeout=30.0):
                        raise TimeoutError(f"no command for tick {k}")
                    v = self._cmds.pop(k)
                status = self._status.get(k, {})
                if bpos is not None:
                    smin = segment_min_distance(pos, v, bpos, ball.vel(t), dt)
                    if smin < sep_min:
                        sep_min, sep_min_t = smin, t
                    hit = hit or sep_min < e.uav_radius + e.ball_radius
                rows.append([t, *pos, *v, *(bpos if bpos is not None else (np.nan,) * 3)])
                if self.rt > 0:
                    lag = t / self.rt - (time.time() - wall0)
                    if lag > 0:
                        time.sleep(lag)
                if status.get("phase") == "goal" or t >= t_max or (hit and t > sep_min_t + 0.5):
                    break
                pos = pos + v * dt                                    # Eq. 2
                k += 1
        finally:
            if hasattr(renderer, "close"):
                renderer.close()
        commit = next((s for s in self._status.values() if s.get("commit")), None)
        ci = commit["commit"] if commit else None
        summary = dict(name=self.exp, backend=self.kind, via="ros2", ticks=k + 1,
                       commit_t=ci["t"] if ci else None, commit_d=ci["d"] if ci else None,
                       tau_hat=ci["tau"] if ci else None,
                       sep_min=None if not ball.present else round(sep_min, 4), sep_min_t=sep_min_t,
                       collision=hit, goal_t=next((s["goal_t"] for s in self._status.values() if s.get("goal_t")), None),
                       wall_s=round(time.time() - wall0, 2))
        self.out.mkdir(parents=True, exist_ok=True)
        (self.out / "summary.json").write_text(json.dumps(summary, indent=1))
        np.savetxt(self.out / "trajectory.csv", np.array(rows), delimiter=",", fmt="%.6f",
                   header="t,x,y,z,vx,vy,vz,bx,by,bz", comments="")
        self.pub_done.publish(String(data=json.dumps(summary)))
        self.get_logger().info("done: " + json.dumps(summary))
        return summary


def main():
    rclpy.init()
    node = SimNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        node.run()
        time.sleep(1.0)            # let the summary and last markers go out
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
