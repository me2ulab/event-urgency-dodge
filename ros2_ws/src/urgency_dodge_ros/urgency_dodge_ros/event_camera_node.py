"""ESIM event camera node: camera frames -> events (Eqs. 6-11) -> 50 ms window (Eq. 12).

Publishes, stamped with the frame's tick:
  /dodge/events/count   32SC1  per-pixel event count in the window (what perception reads)
  /dodge/events/image   rgb8   net polarity (red ON, blue OFF) for RViz
"""
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .common import LATCHED_QOS, LOCKSTEP_QOS, from_image, polarity_rgb, tick_of, tick_time, to_image

from urgency_dodge.loop import EventFront  # noqa: E402
from experiments.run import experiments  # noqa: E402


class EventCameraNode(Node):
    def __init__(self):
        super().__init__("dodge_event_camera")
        self.declare_parameter("experiment", "closed_loop_ttc_check")
        _, self.P = experiments()[self.get_parameter("experiment").value]
        self.front = EventFront(self.P)
        self.pub_n = self.create_publisher(Image, "/dodge/events/count", LOCKSTEP_QOS)
        self.pub_v = self.create_publisher(Image, "/dodge/events/image", 10)
        self.create_subscription(Image, "/dodge/camera/image_raw", self.on_image, LOCKSTEP_QOS)
        self.last_k = -1

    def on_image(self, m: Image):
        k = tick_of(m.header.stamp)
        if k <= self.last_k:                 # a restarted sim begins again at tick 0
            self.front = EventFront(self.P)
        self.last_k = k
        img = from_image(m)
        _, counts = self.front.step(img, tick_time(k))
        self.pub_n.publish(to_image(counts.astype("int32"), "32SC1", m.header.stamp))
        cam = self.P.camera
        self.pub_v.publish(to_image(polarity_rgb(self.front.win.polarity_image(cam.width, cam.height)),
                                    "rgb8", m.header.stamp))


def main():
    rclpy.init()
    n = EventCameraNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
