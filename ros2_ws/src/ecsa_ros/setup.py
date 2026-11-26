from glob import glob
from pathlib import Path
from setuptools import setup

# the nodes import ecsa_core/sims/experiments from the repository root; record where it is
ROOT = Path(__file__).resolve().parents[3]
(Path(__file__).parent / "ecsa_ros" / "_root.py").write_text(f"ROOT = {str(ROOT)!r}\n")

setup(
    name="ecsa_ros",
    version="1.0.0",
    packages=["ecsa_ros"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/ecsa_ros"]),
        ("share/ecsa_ros", ["package.xml"]),
        ("share/ecsa_ros/launch", glob("launch/*.py")),
        ("share/ecsa_ros/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "sim_node = ecsa_ros.sim_node:main",
        "event_camera_node = ecsa_ros.event_camera_node:main",
        "avoidance_node = ecsa_ros.avoidance_node:main",
        "viz_node = ecsa_ros.viz_node:main",
    ]},
)
