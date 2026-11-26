from glob import glob
from pathlib import Path
from setuptools import setup

# the nodes import urgency_dodge/sims/experiments from the repository root; record where it is
ROOT = Path(__file__).resolve().parents[3]
(Path(__file__).parent / "urgency_dodge_ros" / "_root.py").write_text(f"ROOT = {str(ROOT)!r}\n")

setup(
    name="urgency_dodge_ros",
    version="1.0.0",
    packages=["urgency_dodge_ros"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/urgency_dodge_ros"]),
        ("share/urgency_dodge_ros", ["package.xml"]),
        ("share/urgency_dodge_ros/launch", glob("launch/*.py")),
        ("share/urgency_dodge_ros/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "sim_node = urgency_dodge_ros.sim_node:main",
        "event_camera_node = urgency_dodge_ros.event_camera_node:main",
        "avoidance_node = urgency_dodge_ros.avoidance_node:main",
        "viz_node = urgency_dodge_ros.viz_node:main",
    ]},
)
