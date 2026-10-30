"""Bring up communication + vision + master together.

    ros2 launch master autonomous.launch.py

Override anything from the command line:

    ros2 launch master autonomous.launch.py steer_gain:=-18.0 max_pwm:=90.0
    ros2 launch master autonomous.launch.py mode:=goto_point target_x:=15.0

The teleop node is deliberately NOT launched here. It needs a terminal of its
own for raw keyboard input, and it would fight master_node for /cmd_drive.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("remote_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("send_port", default_value="12151"),
        DeclareLaunchArgument("recv_port", default_value="12152"),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/catvehicle/camera_front/image_raw_front",
        ),
        DeclareLaunchArgument("mode", default_value="follow_lane"),
        DeclareLaunchArgument("steer_gain", default_value="-25.0"),
        DeclareLaunchArgument("max_pwm", default_value="120.0"),
        DeclareLaunchArgument("min_pwm", default_value="60.0"),
        DeclareLaunchArgument("target_x", default_value="10.0"),
        DeclareLaunchArgument("target_y", default_value="0.0"),
    ]

    communication = Node(
        package="communication",
        executable="communication_node",
        name="communication_node",
        output="screen",
        parameters=[{
            "remote_host": LaunchConfiguration("remote_host"),
            "send_port": LaunchConfiguration("send_port"),
            "recv_port": LaunchConfiguration("recv_port"),
        }],
    )

    vision = Node(
        package="vision",
        executable="vision_node",
        name="vision_node",
        output="screen",
        parameters=[{
            "image_topic": LaunchConfiguration("image_topic"),
        }],
    )

    master = Node(
        package="master",
        executable="master_node",
        name="master_node",
        output="screen",
        parameters=[{
            "mode": LaunchConfiguration("mode"),
            "steer_gain": LaunchConfiguration("steer_gain"),
            "max_pwm": LaunchConfiguration("max_pwm"),
            "min_pwm": LaunchConfiguration("min_pwm"),
            "target_x": LaunchConfiguration("target_x"),
            "target_y": LaunchConfiguration("target_y"),
        }],
    )

    return LaunchDescription(args + [communication, vision, master])
