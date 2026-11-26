#!/bin/bash
# Reproduce everything: build, tests, all experiments (both simulators), figures and REPORT.md.
# Runs inside the sim container (PX4/Gazebo Jetty/ROS 2 Lyrical userspace):
#     ./run_all.sh            # ~5 min
#     ./run_all.sh --ros      # also the ROS 2 == offline equivalence tests (+2 min)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENTER="${ENTER:-$HERE/../uav_event_nav/container/enter.sh}"
ROS_TESTS=0; [[ "${1:-}" == "--ros" ]] && ROS_TESTS=1
"$ENTER" -- "set -e; cd '$HERE'
  python3 gazebo/gen_world.py
  (cd ros2_ws && colcon build)
  python3 -m pytest tests -q
  python3 experiments/run.py --suite raytrace
  python3 experiments/run.py --suite gazebo
  python3 experiments/make_figures.py > /dev/null
  if [ $ROS_TESTS = 1 ]; then ECSA_ROS_TESTS=1 python3 -m pytest tests/test_ros_equivalence.py -q; fi
  echo; echo 'done: results/REPORT.md, results/figures/'"
