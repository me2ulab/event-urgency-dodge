"""Gazebo lockstep renderer: the camera frame for tick k is rendered from the commanded pose.

Per tick: set the poses of `event_cam` and `ball` (one set_pose_vector request), advance the paused
world by exactly one 25 ms step, and block until the camera frame produced by that step arrives.
The vehicle is NOT simulated by Gazebo physics (paper Sec. 3.1: both plants integrate the
commanded pose); Gazebo is the renderer of the brick corridor.
"""
from __future__ import annotations

import os
import subprocess
import threading
import time
from pathlib import Path

import numpy as np
from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.image_pb2 import Image
from gz.msgs.pose_v_pb2 import Pose_V
from gz.msgs.world_control_pb2 import WorldControl
from gz.transport import Node

from urgency_dodge.params import Params

ROOT = Path(__file__).resolve().parents[1]
WORLD_FILE = ROOT / "gazebo" / "worlds" / "dodge_corridor.sdf"
WORLD = "dodge_corridor"
PARK = (0.0, 0.0, -50.0)            # ball position when absent (below the floor, out of view)


class GazeboLockstepRenderer:
    name = "gazebo"

    def __init__(self, P: Params, start_server: bool = True, gui: bool = False, timeout_s: float = 60.0):
        self.P = P
        self.proc = None
        if start_server:
            if not WORLD_FILE.exists():
                subprocess.run(["python3", str(ROOT / "gazebo" / "gen_world.py")], check=True)
            cmd = ["gz", "sim", "-v", "2", str(WORLD_FILE)] + ([] if gui else ["-s"])
            log = open(ROOT / "results" / "gz_server.log", "w") if (ROOT / "results").exists() else subprocess.DEVNULL
            self.proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy())
        self.node = Node()
        self._cv = threading.Condition()
        self._img, self._count = None, 0
        self.node.subscribe(Image, "/dodge/camera", self._on_image)
        self._wait_service(f"/world/{WORLD}/set_pose_vector", timeout_s)
        self._wait_service(f"/world/{WORLD}/control", timeout_s)
        self.frames = 0
        # the services are advertised before the world accepts commands: retry the first pose update
        t0 = time.time()
        while True:
            try:
                self._set_poses((P.encounter.x0, 0, P.encounter.z_pad), PARK)
                break
            except RuntimeError:
                if time.time() - t0 > timeout_s:
                    raise
                time.sleep(0.5)

    # ---------------------------------------------------------------------------------------
    def _wait_service(self, name, timeout_s):
        t0 = time.time()
        while name not in self.node.service_list():
            if time.time() - t0 > timeout_s:
                raise TimeoutError(f"gz service {name} not available (is the server up?)")
            time.sleep(0.2)

    def _on_image(self, msg: Image):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        gray = (0.299 * a[..., 0] + 0.587 * a[..., 1] + 0.114 * a[..., 2]).round().astype(np.uint8)
        with self._cv:
            self._img, self._count = gray, self._count + 1
            self._cv.notify_all()

    def _set_poses(self, cam_xyz, ball_xyz):
        req = Pose_V()
        for name, (x, y, z) in (("event_cam", cam_xyz), ("ball", ball_xyz)):
            p = req.pose.add()
            p.name = name
            p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
            p.orientation.w = 1.0
        for attempt in range(20):
            ok, rep = self.node.request(f"/world/{WORLD}/set_pose_vector", req, Pose_V, Boolean, 5000)
            if ok and rep.data:
                return
            time.sleep(0.05)
        raise RuntimeError(f"set_pose_vector failed (ok={ok}) for cam={cam_xyz} ball={ball_xyz}")

    def _step(self, n: int):
        wc = WorldControl()
        wc.pause = True
        wc.multi_step = n
        ok, rep = self.node.request(f"/world/{WORLD}/control", wc, WorldControl, Boolean, 5000)
        if not (ok and rep.data):
            raise RuntimeError("world step failed")

    # The camera frame published for a step shows the scene as it was BEFORE that step's pose update
    # (measured: experiments/probe_gazebo.py), so each tick advances two steps and keeps the second frame.
    STEPS_PER_TICK = 2

    def render(self, uav_pos, ball_pos, t, timeout_s: float = 30.0):
        self._set_poses(uav_pos, ball_pos if ball_pos is not None else PARK)
        with self._cv:
            n0 = self._count
        self._step(self.STEPS_PER_TICK)
        with self._cv:
            if not self._cv.wait_for(lambda: self._count >= n0 + self.STEPS_PER_TICK, timeout=timeout_s):
                raise TimeoutError(f"no camera frame for t={t:.3f}")
            img = self._img.copy()
        self.frames += 1
        return img

    def close(self):
        if self.proc is not None:
            self.proc.terminate()
            try:
                self.proc.wait(10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
            self.proc = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
