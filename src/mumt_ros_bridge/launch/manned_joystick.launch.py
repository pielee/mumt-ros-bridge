"""
manned_joystick.launch.py
-------------------------
Full manned-aircraft joystick control stack:
  joy_node (joy)  ->  mumt_joystick (this pkg)  ->  /mumt/aircraft_commands
                                                ->  bridge (this pkg)  ->  UDP 5005 -> Unreal

Run:  ros2 launch mumt_ros_bridge manned_joystick.launch.py
      (set start_bridge:=false if you already run the bridge elsewhere)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("mumt_ros_bridge")
    default_params = os.path.join(share, "config", "joystick.yaml")

    params_file = LaunchConfiguration("params_file")
    start_bridge = LaunchConfiguration("start_bridge")
    unreal_ip = LaunchConfiguration("unreal_ip")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="joystick + joy_node parameter file.",
        ),
        DeclareLaunchArgument(
            "start_bridge",
            default_value="true",
            description="Also start bridge_node (UDP<->ROS). Set false if running it separately.",
        ),
        DeclareLaunchArgument(
            "unreal_ip",
            default_value="127.0.0.1",
            description="IP of the machine running Unreal. Set to the PC's IP if ROS runs on a "
                        "separate box (e.g. the Jetson) from the simulator.",
        ),

        # Joystick driver -> sensor_msgs/Joy on /joy
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[params_file],
            output="screen",
        ),

        # Joy -> JSON command on /mumt/aircraft_commands
        Node(
            package="mumt_ros_bridge",
            executable="joystick",
            name="mumt_joystick",
            parameters=[params_file],
            output="screen",
        ),

        # ROS <-> UDP bridge (optional)
        Node(
            package="mumt_ros_bridge",
            executable="bridge",
            name="mumt_bridge",
            output="screen",
            parameters=[{"unreal_ip": unreal_ip}],
            condition=IfCondition(start_bridge),
        ),
    ])
