from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    rviz = os.path.join(
        get_package_share_directory("ibvs_demo"),
        "rviz",
        "ibvs.rviz"
    )

    return LaunchDescription([

        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     output="screen",
        #     arguments=[
        #         "-d",
        #         rviz
        #     ]
        # ),

        Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            output="screen"
        )

    ])