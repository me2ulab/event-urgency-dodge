"""The ROS 2 pipeline must reproduce the offline runner exactly (same core code, lockstep).

Needs the built workspace (ros2_ws) and takes ~1 min per backend, so it is opt-in:
    ECSA_ROS_TESTS=1 python3 -m pytest tests/test_ros_equivalence.py -q
"""
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
pytestmark = pytest.mark.skipif(os.environ.get("ECSA_ROS_TESTS") != "1", reason="set ECSA_ROS_TESTS=1")

KEYS = ("commit_t", "commit_d", "tau_hat", "sep_min", "collision", "goal_t")


@pytest.mark.parametrize("exp", ["closed_loop_ttc_check", "gz_closed_015"])
def test_ros_equals_offline(exp, tmp_path):
    from experiments.run import run
    offline = run(exp)
    cmd = (f". {ROOT}/ros2_ws/install/setup.bash && ros2 launch ecsa_ros ecsa.launch.py experiment:={exp} "
           f"rviz:=false realtime:=0 exit_when_done:=true")
    subprocess.run(["bash", "-lc", cmd], check=True, timeout=300, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    ros = json.loads((ROOT / "results" / "ros" / exp / "summary.json").read_text())
    for k in KEYS:
        assert ros[k] == pytest.approx(offline[k], abs=1e-6) if isinstance(offline[k], float) else ros[k] == offline[k], k
