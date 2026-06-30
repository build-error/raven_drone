from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package="ibvs_mpc",
            executable="aruco_detector",
            output="screen"
        ),

        Node(
            package="ibvs_mpc",
            executable="ibvs_controller",
            output="screen"
        )

    ])