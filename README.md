# Event-camera collision-urgency estimation and reactive avoidance

Code and data for *Event-Camera Sensor-Driven Collision Urgency Estimation and Reactive Avoidance for
Autonomous UAV Navigation* (I. Akanbi and M. Ayomoh, University of Pretoria).

A UAV cruises down a 28 m × 4 m corridor at 1.5 m altitude. A 10 m/s ball is thrown straight at it. A single
forward event camera (160 × 120, 40 Hz) detects the ball in a 16 px centre crop, confirms that its apparent
diameter is growing, and estimates time-to-contact τ = d/ḋ. After three passing ticks it sidesteps by
v_y = Δy/τ, then returns to the centreline and continues to the goal. No depth sensor or network is used, and
the ball's pose never reaches perception.

The method is flown in two simulators: a ray-traced corridor and Gazebo. Both run natively in ROS 2 with RViz
visualisation. [IMPLEMENTATION_NOTES.md](IMPLEMENTATION_NOTES.md) lists every parameter and the implementation
choices the paper leaves open. [DATA.md](DATA.md) maps each reported result to its data.

## Layout

| Path | What |
|---|---|
| `urgency_dodge/` | The method, pure numpy: parameters (`params.py`, each tied to its equation), ESIM events (Eqs. 6-12), foveal detection (13-21), looming tracker / τ / commit gate (22-31), sidestep controller, plant and ball (1-2, 32-38), closed loop and scoring (39-41) |
| `sims/raytrace.py` | Ray-traced corridor: axial stripes, end-wall checkerboard, dark shelf spheres, checker-patterned ball |
| `sims/gz_lockstep.py` | Gazebo as a lockstep renderer: set poses → step → camera frame of exactly that pose |
| `gazebo/gen_world.py` | Generates the brick-corridor world (`gazebo/worlds/dodge_corridor.sdf`) and its textures |
| `ros2_ws/src/urgency_dodge_ros/` | ROS 2 nodes: `sim_node` (clock, plant, renderer), `event_camera_node`, `avoidance_node`, `viz_node`; launch file and RViz config |
| `experiments/run.py` | All reported runs, by name (`--list`) |
| `experiments/make_figures.py` | The figures and `results/REPORT.md` |
| `results/` | Data of every reported run (see DATA.md) |
| `tests/` | 16 unit and closed-loop tests, plus an opt-in ROS-equals-offline test |

Equation numbers in the code follow the full-length method description (Eqs. 1-41).

## Requirements

ROS 2 Lyrical, Gazebo Jetty (gz-sim 10) with its Python bindings, and Python 3 with numpy, scipy, matplotlib,
OpenCV and pytest. On the authors' machine these run in an Ubuntu 26.04 userspace, entered with
`uav_event_nav/container/enter.sh`. `run_all.sh` uses that path unless `ENTER` is set. Set `ENTER=env` on a
machine with the stack installed natively.

## Run

```bash
./run_all.sh                 # build, tests, all experiments, figures, REPORT.md (~5 min)
./run_all.sh --ros           # plus the ROS 2 == offline equivalence tests

python3 experiments/run.py --list
python3 experiments/run.py gz_closed_015 -v
```

Watch a closed-loop run in RViz, and optionally the Gazebo GUI:

```bash
cd ros2_ws && colcon build && . install/setup.bash
ros2 launch urgency_dodge_ros dodge.launch.py experiment:=gz_closed_015 gz_gui:=true
# experiment:=closed_loop_ttc_check (ray-traced), realtime:=0.5 (slow motion), rviz:=false
```

RViz shows the corridor, shelving, UAV and ball with their paths, a status label (phase, d, τ, H), the onboard
camera, the 50 ms event image, and the foveal crop with the components and the detection.

## Results

Summary in [results/REPORT.md](results/REPORT.md), next to the values printed in the paper:

* **Speed sweep (ray-traced):** commit times are within one camera tick (25 ms) of the paper for all six
  configurations. Every throw is cleared, including 12 m/s at 1.5 m/s cruise.
* **Gazebo, nominal encounter:** dodge with a 0.87 m miss, goal reached at 15.75 s.
* **Controls:** the empty corridor never dodges, and avoidance off is always hit, in both simulators.
* **Timing:** command-path time is 0.5 ms per tick, against a 25 ms camera period.
* **Equivalence:** ROS 2 runs reproduce the offline runner exactly, for both simulators.
