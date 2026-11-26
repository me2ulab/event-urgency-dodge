"""Generate the Gazebo brick corridor (manuscript Figs. 3-4, 6) and its textures.

    python3 gazebo/gen_world.py            # writes gazebo/worlds/ecsa_corridor.sdf + gazebo/textures/*.png

Layout follows the paper: 28 m x 4 m corridor, 1.5 m working height, goal at x = 24 m.
Appearance follows the surviving Gazebo renders (PHD/fig5_gz_image_events.png): brick walls,
side shelving blocks, a dark ceiling with light fixtures, a plank floor and a spotted ball.
RECONSTRUCTED: the original SDF and textures were lost; every size and texture below is chosen
to match those renders, not copied. Textures are generated with a fixed seed (deterministic).
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from ecsa_core.params import Params  # noqa: E402

TEX = HERE / "textures"
WORLD = HERE / "worlds" / "ecsa_corridor.sdf"
WORLD_NAME = "ecsa_corridor"
PX_PER_M = 96


def _rng(seed):
    return np.random.default_rng(seed)


def brick_texture(w_m, h_m, seed, base=(0.55, 0.85), brick=(0.50, 0.20), mortar=0.025):
    """Running-bond brick with per-brick tone, speckle and grey mortar (grayscale, uint8 RGB)."""
    r = _rng(seed)
    W, H = int(w_m * PX_PER_M), int(h_m * PX_PER_M)
    img = np.zeros((H, W), np.float32)
    bw, bh, mt = int(brick[0] * PX_PER_M), int(brick[1] * PX_PER_M), max(1, int(mortar * PX_PER_M))
    for row, y0 in enumerate(range(0, H, bh)):
        off = (bw // 2) * (row % 2)
        for x0 in range(-off, W, bw):
            tone = r.uniform(*base)
            x1, y1 = max(0, x0 + mt), min(H, y0 + bh)
            img[y0 + mt:y1, x1:min(W, x0 + bw)] = tone
    img += r.normal(0, 0.05, img.shape).astype(np.float32)
    img = cv2.GaussianBlur(img, (0, 0), 0.8)
    img[img == 0] = 0.35
    return _rgb(img)


def mottled_texture(w_m, h_m, seed, mean=0.32, amp=0.12):
    """Dark, low-frequency mottled surface (the shelving blocks)."""
    r = _rng(seed)
    W, H = int(w_m * PX_PER_M), int(h_m * PX_PER_M)
    img = np.zeros((H, W), np.float32)
    for s, a in ((48, 1.0), (16, 0.6), (5, 0.35)):
        n = r.normal(0, 1, (max(2, H // s), max(2, W // s))).astype(np.float32)
        img += a * cv2.resize(n, (W, H), interpolation=cv2.INTER_CUBIC)
    img = mean + amp * img / (np.abs(img).max() + 1e-6)
    return _rgb(img)


def plank_texture(w_m, h_m, seed, plank=0.15):
    """Floor planks running along x (texture u = corridor x)."""
    r = _rng(seed)
    W, H = int(w_m * PX_PER_M), int(h_m * PX_PER_M)
    img = np.zeros((H, W), np.float32)
    ph = int(plank * PX_PER_M)
    for y0 in range(0, H, ph):
        x = -int(r.uniform(0, 2.0) * PX_PER_M)
        while x < W:
            L = int(r.uniform(1.0, 3.0) * PX_PER_M)
            img[y0 + 1:y0 + ph, max(0, x + 1):min(W, x + L)] = r.uniform(0.40, 0.62)
            x += L
    grain = r.normal(0, 1, (H, max(2, W // 24))).astype(np.float32)
    img += 0.04 * cv2.resize(grain, (W, H), interpolation=cv2.INTER_LINEAR)
    img[img == 0] = 0.25
    return _rgb(img)


def ceiling_texture(w_m, h_m, seed):
    r = _rng(seed)
    W, H = int(w_m * PX_PER_M), int(h_m * PX_PER_M)
    img = 0.10 + 0.03 * r.normal(0, 1, (H, W)).astype(np.float32)
    for x in range(0, W, int(0.6 * PX_PER_M)):           # ceiling battens across the corridor
        img[:, x:x + 3] = 0.22
    return _rgb(cv2.GaussianBlur(img, (0, 0), 0.7))


def ball_texture(seed=7):
    """Equirectangular map: white ball with black spots (the 'spotted ball')."""
    r = _rng(seed)
    W, H = 1024, 512
    img = np.full((H, W), 0.92, np.float32)
    lon = (np.arange(W) + 0.5) / W * 2 * np.pi - np.pi
    lat = np.pi / 2 - (np.arange(H) + 0.5) / H * np.pi
    LON, LAT = np.meshgrid(lon, lat)
    P = np.stack([np.cos(LAT) * np.cos(LON), np.cos(LAT) * np.sin(LON), np.sin(LAT)], -1)
    # 12 spot centres on an icosahedron (like a football's pentagons), angular radius 0.36 rad
    g = (1 + 5 ** 0.5) / 2
    C = np.array([(0, 1, g), (0, -1, g), (0, 1, -g), (0, -1, -g), (1, g, 0), (-1, g, 0), (1, -g, 0),
                  (-1, -g, 0), (g, 0, 1), (-g, 0, 1), (g, 0, -1), (-g, 0, -1)], float)
    C /= np.linalg.norm(C, axis=1, keepdims=True)
    ang = np.arccos(np.clip(P @ C.T, -1, 1)).min(-1)
    img[ang < 0.36] = 0.06
    return _rgb(img)


def _rgb(gray):
    g = (np.clip(gray, 0, 1) * 255).astype(np.uint8)
    return np.stack([g, g, g], -1)


def _material(tex_path: Path, emissive: float = 0.0) -> str:
    em = f"<emissive>{emissive} {emissive} {emissive} 1</emissive>" if emissive else ""
    return (f"<material><diffuse>1 1 1 1</diffuse><ambient>0.6 0.6 0.6 1</ambient><specular>0.05 0.05 0.05 1</specular>{em}"
            f"<pbr><metal><albedo_map>{tex_path}</albedo_map><roughness>0.9</roughness><metalness>0</metalness>"
            f"</metal></pbr></material>")


def _box(name, pos, size, tex: Path, emissive=0.0):
    x, y, z = pos
    sx, sy, sz = size
    return (f"<model name='{name}'><static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose><link name='l'>"
            f"<visual name='v'><cast_shadows>false</cast_shadows><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>"
            f"{_material(tex, emissive)}</visual></link></model>")


def build(P: Params = Params()) -> str:
    co = P.corridor
    L = co.x_max - co.x_min
    xc = 0.5 * (co.x_min + co.x_max)
    hw, Hc, th = co.half_width, co.ceiling, 0.2
    TEX.mkdir(parents=True, exist_ok=True)
    # one texture per face size so a box face maps it 1:1 (box UVs span each face once)
    tex = {}
    def put(name, img):
        p = TEX / f"{name}.png"; cv2.imwrite(str(p), img); tex[name] = p
    put("wall_left", brick_texture(L, Hc, 1)); put("wall_right", brick_texture(L, Hc, 2))
    put("end_wall", brick_texture(2 * hw, Hc, 3)); put("back_wall", brick_texture(2 * hw, Hc, 4))
    put("floor", plank_texture(L, 2 * hw, 5)); put("ceiling", ceiling_texture(L, 2 * hw, 6))
    put("shelf", mottled_texture(2.5, 1.6, 8)); put("ball", ball_texture())
    put("light", _rgb(np.full((16, 16), 1.0, np.float32)))
    parts = [
        _box("wall_left", (xc, hw + th / 2, Hc / 2), (L, th, Hc), tex["wall_left"]),
        _box("wall_right", (xc, -hw - th / 2, Hc / 2), (L, th, Hc), tex["wall_right"]),
        _box("end_wall", (co.x_max + th / 2, 0, Hc / 2), (th, 2 * hw, Hc), tex["end_wall"]),
        _box("back_wall", (co.x_min - th / 2, 0, Hc / 2), (th, 2 * hw, Hc), tex["back_wall"]),
        _box("floor", (xc, 0, -th / 2), (L, 2 * hw, th), tex["floor"]),
        _box("ceiling", (xc, 0, Hc + th / 2), (L, 2 * hw, th), tex["ceiling"]),
    ]
    # shelving blocks against both walls, 0.40 m deep, 1.6 m tall, 2.5 m long, 2 m gaps (brick shows above/between)
    k = 0
    for x0 in np.arange(2.0, co.x_max - 2.0, 4.5):
        for s in (+1, -1):
            parts.append(_box(f"shelf_{k}", (x0 + 1.25, s * (hw - 0.20), 0.8), (2.5, 0.40, 1.6), tex["shelf"])); k += 1
    # ceiling light fixtures (emissive) and point lights
    lights = []
    for i, x in enumerate(np.arange(0.0, co.x_max, 4.0)):
        parts.append(_box(f"lamp_{i}", (x, 0, Hc - 0.03), (1.2, 0.25, 0.04), tex["light"], emissive=1.0))
        lights.append(f"<light type='point' name='pl_{i}'><pose>{x:.2f} 0 {Hc - 0.15:.2f} 0 0 0</pose>"
                      f"<diffuse>0.9 0.9 0.85 1</diffuse><specular>0.1 0.1 0.1 1</specular>"
                      f"<attenuation><range>12</range><constant>0.3</constant><linear>0.12</linear><quadratic>0.02</quadratic></attenuation>"
                      f"<cast_shadows>false</cast_shadows></light>")
    e, c, cam = P.encounter, P.control, P.camera
    R = e.ball_radius
    ball = (f"<model name='ball'><pose>{e.x_launch} 0 {c.z_cruise} 0 0 0</pose><link name='l'><gravity>false</gravity>"
            f"<kinematic>true</kinematic><inertial><mass>0.058</mass></inertial>"
            f"<visual name='v'><cast_shadows>false</cast_shadows><geometry><sphere><radius>{R}</radius></sphere></geometry>"
            f"{_material(tex['ball'])}</visual></link></model>")
    body = ("".join(f"<visual name='arm{i}'><pose>0 0 0 0 0 {0.785 + 1.5708 * i:.4f}</pose><geometry><box><size>0.42 0.03 0.02</size></box></geometry>"
                    f"<material><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>" for i in range(2))
            + "<visual name='hub'><geometry><box><size>0.12 0.12 0.05</size></box></geometry><material><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>")
    camera = (f"<model name='event_cam'><pose>{e.x0} 0 {e.z_pad} 0 0 0</pose><link name='l'><gravity>false</gravity>"
              f"<kinematic>true</kinematic><inertial><mass>1.5</mass></inertial>{body}"
              f"<sensor name='cam' type='camera'><always_on>1</always_on><update_rate>{cam.rate_hz}</update_rate>"
              f"<topic>ecsa/camera</topic><camera><horizontal_fov>{cam.hfov:.6f}</horizontal_fov>"
              f"<image><width>{cam.width}</width><height>{cam.height}</height><format>R8G8B8</format></image>"
              f"<clip><near>0.3</near><far>60</far></clip></camera></sensor></link></model>")
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{WORLD_NAME}">
    <physics type="ode"><max_step_size>{cam.dt}</max_step_size><real_time_factor>0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <gravity>0 0 0</gravity>
    <scene><grid>false</grid><ambient>0.35 0.35 0.35 1</ambient><background>0 0 0 1</background><shadows>false</shadows></scene>
    <light type="directional" name="fill"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.25 0.25 0.25 1</diffuse><specular>0 0 0 1</specular><direction>0.3 0.2 -0.9</direction></light>
    {''.join(lights)}
    {''.join(parts)}
    {ball}
    {camera}
  </world>
</sdf>
"""


if __name__ == "__main__":
    WORLD.parent.mkdir(parents=True, exist_ok=True)
    WORLD.write_text(build())
    print("wrote", WORLD, "and", len(list(TEX.glob('*.png'))), "textures in", TEX)
