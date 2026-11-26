"""RViz visualisation: corridor geometry (latched), UAV and ball markers, flown paths, status text."""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from .common import LATCHED_QOS, LOCKSTEP_QOS

from experiments.run import experiments  # noqa: E402


def _rgba(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


class VizNode(Node):
    def __init__(self):
        super().__init__("dodge_viz")
        self.declare_parameter("experiment", "closed_loop_ttc_check")
        self.kind, self.P = experiments()[self.get_parameter("experiment").value]
        self.pub_static = self.create_publisher(MarkerArray, "/dodge/markers/world", LATCHED_QOS)
        self.pub_dyn = self.create_publisher(MarkerArray, "/dodge/markers/agents", 10)
        self.pub_upath = self.create_publisher(Path, "/dodge/uav/path", 10)
        self.pub_bpath = self.create_publisher(Path, "/dodge/ball/path", 10)
        self.create_subscription(PoseStamped, "/dodge/uav/pose", self.on_uav, LOCKSTEP_QOS)
        self.create_subscription(PoseStamped, "/dodge/ball/pose", self.on_ball, LOCKSTEP_QOS)
        self.create_subscription(String, "/dodge/avoidance/status", self.on_status, LOCKSTEP_QOS)
        self.upath, self.bpath = Path(), Path()
        self.upath.header.frame_id = self.bpath.header.frame_id = "world"
        self.status = {}
        self.ball = None
        self.publish_world()

    def _m(self, ns, i, typ, frame="world"):
        m = Marker()
        m.header.frame_id, m.ns, m.id, m.type, m.action = frame, ns, i, typ, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def publish_world(self):
        co, P = self.P.corridor, self.P
        L, xc, hw, H = co.x_max - co.x_min, 0.5 * (co.x_min + co.x_max), co.half_width, co.ceiling
        ma, i = MarkerArray(), 0
        for name, (x, y, z), (sx, sy, sz), col in [
            ("walls", (xc, hw + 0.05, H / 2), (L, 0.1, H), (0.55, 0.45, 0.40, 0.35)),
            ("walls", (xc, -hw - 0.05, H / 2), (L, 0.1, H), (0.55, 0.45, 0.40, 0.35)),
            ("walls", (co.x_max + 0.05, 0, H / 2), (0.1, 2 * hw, H), (0.55, 0.45, 0.40, 0.35)),
            ("floor", (xc, 0, -0.01), (L, 2 * hw, 0.02), (0.35, 0.35, 0.35, 1.0))]:
            m = self._m(name, i, Marker.CUBE); i += 1
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
            m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
            m.color = _rgba(*col)
            ma.markers.append(m)
        if self.kind == "raytrace":
            from sims.raytrace import Look
            lk = Look()
            for x in lk.shelf_xs:
                for s in (1, -1):
                    m = self._m("shelf_spheres", i, Marker.SPHERE); i += 1
                    m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(s * lk.shelf_y), float(lk.shelf_z)
                    m.scale.x = m.scale.y = m.scale.z = float(2 * lk.shelf_r)
                    m.color = _rgba(0.05, 0.05, 0.05)
                    ma.markers.append(m)
        else:
            for x0 in np.arange(2.0, co.x_max - 2.0, 4.5):
                for s in (1, -1):
                    m = self._m("shelves", i, Marker.CUBE); i += 1
                    m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x0 + 1.25), float(s * (hw - 0.2)), 0.8
                    m.scale.x, m.scale.y, m.scale.z = 2.5, 0.4, 1.6
                    m.color = _rgba(0.25, 0.25, 0.27)
                    ma.markers.append(m)
        c, e = P.control, P.encounter
        g = self._m("goal", i, Marker.CYLINDER); i += 1
        g.pose.position.x, g.pose.position.z = float(c.goal_x), float(c.z_cruise)
        g.scale.x = g.scale.y = float(2 * c.goal_tol); g.scale.z = 0.02
        g.color = _rgba(0.18, 0.53, 0.87, 0.6)
        pad = self._m("pad", i, Marker.CYLINDER); i += 1
        pad.pose.position.x, pad.pose.position.z = float(e.x0), float(e.z_pad / 2)
        pad.scale.x = pad.scale.y = 0.6; pad.scale.z = float(e.z_pad)
        pad.color = _rgba(0.2, 0.2, 0.2)
        line = self._m("centreline", i, Marker.LINE_STRIP); i += 1
        line.points = [Point(x=float(e.x0), z=float(c.z_cruise)), Point(x=float(c.goal_x), z=float(c.z_cruise))]
        line.scale.x = 0.02
        line.color = _rgba(0.6, 0.6, 0.6, 0.8)
        ma.markers += [g, pad, line]
        self.pub_static.publish(ma)

    def on_ball(self, m):
        self.ball = m
        if not math.isnan(m.pose.position.x):
            self.bpath.poses.append(m); self.bpath.header.stamp = m.header.stamp
            self.pub_bpath.publish(self.bpath)

    def on_status(self, m):
        self.status = json.loads(m.data)

    def on_uav(self, m):
        if self.upath.poses and m.header.stamp.sec == 0 and m.header.stamp.nanosec == 0:
            self.upath.poses, self.bpath.poses = [], []       # sim restarted
        self.upath.poses.append(m); self.upath.header.stamp = m.header.stamp
        self.pub_upath.publish(self.upath)
        e = self.P.encounter
        ma = MarkerArray()
        u = self._m("uav", 0, Marker.SPHERE)
        u.pose = m.pose
        u.scale.x = u.scale.y = u.scale.z = float(2 * e.uav_radius)
        ph = self.status.get("phase", "")
        u.color = _rgba(0.85, 0.55, 0.1, 0.9) if ph in ("sidestep", "hold", "return") else _rgba(0.11, 0.5, 0.23, 0.9)
        ma.markers.append(u)
        if self.ball is not None and not math.isnan(self.ball.pose.position.x):
            b = self._m("ball", 1, Marker.SPHERE)
            b.pose = self.ball.pose
            b.scale.x = b.scale.y = b.scale.z = float(2 * e.ball_radius)
            b.color = _rgba(0.75, 0.22, 0.17)
            ma.markers.append(b)
        txt = self._m("status", 2, Marker.TEXT_VIEW_FACING)
        txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = float(m.pose.position.x), 0.0, 2.6
        s = self.status
        tau = s.get("tau")
        d = s.get("d")
        txt.text = (f"t={s.get('t', 0):.3f} s  {ph}\n"
                    + (f"d={d:.0f} px  " if d is not None else "no detection  ")
                    + (f"tau={tau:.2f} s  " if tau is not None else "")
                    + f"H={s.get('H', 0)}")
        txt.scale.z = 0.25
        txt.color = _rgba(1, 1, 1)
        ma.markers.append(txt)
        self.pub_dyn.publish(ma)


def main():
    rclpy.init()
    n = VizNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
