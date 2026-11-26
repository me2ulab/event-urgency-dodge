#!/bin/bash
# Reproduce everything: build, tests, all experiments (both simulators), figures and results/REPORT.md.
#     ./run_all.sh            # ~5 min
#     ./run_all.sh --ros      # also the ROS 2 == offline equivalence tests (+2 min)
# The stack (ROS 2 Lyrical, Gazebo Jetty) is entered with $ENTER, by default the authors' container script;
# ENTER=env runs on a machine with the stack installed natively.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENTER="${ENTER:-$HERE/../uav_event_nav/container/enter.sh}"
ROS_TESTS=0; [[ "${1:-}" == "--ros" ]] && ROS_TESTS=1
CMD="set -e; cd '$HERE'
  python3 gazebo/gen_world.py
  (cd ros2_ws && colcon build)
  python3 -m pytest tests -q
  python3 experiments/run.py --suite raytrace
  python3 experiments/run.py --suite gazebo
  python3 experiments/make_figures.py > /dev/null
  if [ $ROS_TESTS = 1 ]; then ROS_TESTS=1 python3 -m pytest tests/test_ros_equivalence.py -q; fi
  echo; echo 'done: results/REPORT.md, results/figures/'"
if [[ "$ENTER" == "env" ]]; then bash -lc "$CMD"; else "$ENTER" -- "$CMD"; fi
