# Event-camera collision-urgency dodge: reproduction

This repository rebuilds the code behind the ECSA-13 paper *Event-Camera Sensor-Driven Collision Urgency
Estimation and Reactive Avoidance for Autonomous UAV Navigation* (`../manuscript.v4.docx`). The original code was
lost. Everything here is rebuilt from the full-length draft (`../PHD/ecsa_paper.docx`, Eqs. 1-41) and the surviving
figures. [RECOVERY.md](RECOVERY.md) separates what is faithful to the paper from what had to be reconstructed.

A UAV cruises down a 28 m × 4 m corridor at 1.5 m altitude. A 10 m/s ball is thrown straight at it. A single
forward event camera (160 × 120, 40 Hz) detects the ball in a 16 px centre crop, confirms that its apparent
diameter is growing, and estimates time-to-contact τ = d/ḋ. After three passing ticks it sidesteps by
v_y = Δy/τ, then returns to the centreline and continues to the goal. No depth sensor or network is used, and
the ball's pose never reaches perception.

## Layout

| Path | What |
|---|---|
| `ecsa_core/` | The method, pure numpy: parameters (`params.py`, each tied to its equation), ESIM events (Eqs. 6-12), foveal detection (13-21), looming tracker / τ / commit gate (22-31), sidestep controller, plant and ball (1-2, 32-38), closed loop and scoring (39-41) |
| `sims/raytrace.py` | The "custom simulator": ray-traced corridor with axial stripes, end-wall checkerboard, dark shelf spheres, checker ball |
| `sims/gz_lockstep.py` | Gazebo Jetty as a lockstep renderer: set poses → step → camera frame of exactly that pose |
| `gazebo/gen_world.py` | Generates the brick-corridor world (`gazebo/worlds/ecsa_corridor.sdf`) and its textures |
| `ros2_ws/src/ecsa_ros/` | ROS 2 nodes: `sim_node` (clock, plant, renderer), `event_camera_node`, `avoidance_node`, `viz_node`; launch file and RViz config |
| `experiments/run.py` | The paper's named experiments (Table 1 rows, Table 2 sweep) |
| `experiments/make_figures.py` | Figs. 4-7 and `results/REPORT.md` (paper → reproduced) |
| `tests/` | 17 unit/closed-loop tests and an opt-in ROS-equals-offline test |
| `GREEN_BOX_CHECK.md`, `fig7/`, `scripts/` | The manuscript figure checks (Fig. 6 green box, Fig. 7 rebuild) |

## Run

Everything runs in the sim container (Ubuntu 26.04 userspace with ROS 2 Lyrical and Gazebo Jetty):

```bash
./run_all.sh                 # build, tests, all experiments, figures, REPORT.md (~5 min)
./run_all.sh --ros           # plus the ROS 2 == offline equivalence tests

ENTER=../uav_event_nav/container/enter.sh
$ENTER -- 'cd ~/Documents/PHD/ecsa_dodge_repro && python3 experiments/run.py --list'
$ENTER -- 'cd ~/Documents/PHD/ecsa_dodge_repro && python3 experiments/run.py gz_closed_015 -v'
```

Watch a closed-loop run in RViz, and optionally the Gazebo GUI:

```bash
$ENTER -- 'cd ~/Documents/PHD/ecsa_dodge_repro/ros2_ws && . install/setup.bash &&
           ros2 launch ecsa_ros ecsa.launch.py experiment:=gz_closed_015 gz_gui:=true'
# experiment:=closed_loop_ttc_check (ray-traced), realtime:=0.5 (slow motion), rviz:=false
```

RViz shows the corridor, shelving, UAV and ball with their paths, a status label (phase, d, τ, H), the onboard
camera, the 50 ms event image, and the foveal crop with the components and the detection b*.

## Results

See [results/REPORT.md](results/REPORT.md). In short, against `ecsa_paper.docx`:

* **Ray-traced sweep (Table 2):** all six commit times are within one camera tick (25 ms) of the paper. All
  throws are cleared, including 12 m/s at 1.5 m/s cruise.
* **Gazebo nominal (Table 1):** dodge, 0.87 m miss (paper 0.90 m), goal at 15.75 s (paper 15.75 s). Commit is at
  3.85 s against 3.72 s, because the 7 px ball's measured diameter jitters by one reversal (see RECOVERY.md).
* **Controls:** the empty corridor never dodges, and avoidance off is always hit, in both simulators.
* **Timing:** command-path time is 0.5 ms per tick, against a 25 ms budget (paper 1.27 ms).
* **Equivalence:** ROS 2 runs reproduce the offline runner exactly, for both backends (`tests/test_ros_equivalence.py`).
