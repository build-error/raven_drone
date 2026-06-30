from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package="ros_gz_image",
            executable="image_bridge",
            name="image_bridge",
            output="screen",
            arguments=[
                "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image"
            ]
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="camera_info_bridge",
            output="screen",
            arguments=[
                "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info"
                "@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
            ]
        )

    ])