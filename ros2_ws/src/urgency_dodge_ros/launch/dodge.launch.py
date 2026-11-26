"""Closed-loop run in ROS 2, with RViz and (optionally) the Gazebo GUI.

  ros2 launch ecsa_ros ecsa.launch.py                                   # ray-traced nominal, RViz, real time
  ros2 launch ecsa_ros ecsa.launch.py experiment:=gz_closed_015 gz_gui:=true
  ros2 launch ecsa_ros ecsa.launch.py experiment:=closed_loop_nododge rviz:=false realtime:=0
Experiment names: python3 experiments/run.py --list
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, Shutdown, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration as LC, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    exp = LC("experiment")
    common = {"experiment": exp}
    sim = Node(package="ecsa_ros", executable="sim_node", name="ecsa_sim", output="screen",
               parameters=[{**common, "realtime": ParameterValue(LC("realtime"), value_type=float)}])
    return LaunchDescription([
        DeclareLaunchArgument("experiment", default_value="closed_loop_ttc_check"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("realtime", default_value="1.0", description="0 = as fast as possible"),
        DeclareLaunchArgument("gz_gui", default_value="false", description="attach the Gazebo GUI (gazebo backend)"),
        DeclareLaunchArgument("exit_when_done", default_value="false"),
        Node(package="ecsa_ros", executable="event_camera_node", name="ecsa_event_camera", parameters=[common]),
        Node(package="ecsa_ros", executable="avoidance_node", name="ecsa_avoidance", parameters=[common]),
        Node(package="ecsa_ros", executable="viz_node", name="ecsa_viz", parameters=[common]),
        sim,
        Node(package="rviz2", executable="rviz2", name="rviz2", condition=IfCondition(LC("rviz")),
             arguments=["-d", PathJoinSubstitution([FindPackageShare("ecsa_ros"), "rviz", "ecsa.rviz"])]),
        TimerAction(period=6.0, actions=[ExecuteProcess(cmd=["gz", "sim", "-g"], condition=IfCondition(LC("gz_gui")))]),
        RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=[Shutdown()]),
                             condition=IfCondition(LC("exit_when_done"))),
    ])
