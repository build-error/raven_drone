#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/path.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class FrameVisualizer : public rclcpp::Node
{
public:

    FrameVisualizer()
    : Node("frame_visualizer")
    {
        _path.header.frame_id = "map";

        _path_pub =
            create_publisher<nav_msgs::msg::Path>(
                "/vehicle_path",
                10);

        _local_position_sub =
            create_subscription<
                px4_msgs::msg::VehicleLocalPosition>(
                    "/fmu/out/vehicle_local_position_v1",
                    rclcpp::SensorDataQoS(),
                    std::bind(
                        &FrameVisualizer::localPositionCallback,
                        this,
                        std::placeholders::_1));

        _attitude_sub =
            create_subscription<
                px4_msgs::msg::VehicleAttitude>(
                    "/fmu/out/vehicle_attitude",
                    rclcpp::SensorDataQoS(),
                    [this](
                        const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
                    {
                        _attitude = *msg;
                    });

        _tf_broadcaster =
            std::make_unique<
                tf2_ros::TransformBroadcaster>(*this);

        _gz_node.Subscribe(
            "/world/aruco/dynamic_pose/info",
            &FrameVisualizer::poseCallback,
            this);

        RCLCPP_INFO(
            get_logger(),
            "Frame Visualizer Started");
    }

private:

    void poseCallback(
        const gz::msgs::Pose_V & msg)
    {
        for (int i = 0; i < msg.pose_size(); ++i)
        {
            const auto & pose = msg.pose(i);

            if (pose.name() == "base_link")
            {
                _base_pose = pose;
                _have_base_pose = true;
            }
            else if (pose.name() == "camera_link")
            {
                _camera_pose = pose;
                _have_camera_pose = true;
            }
        }
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "base z=%.3f camera z=%.3f",
            _base_pose.position().z(),
            _camera_pose.position().z()
        );
    }

    void localPositionCallback(
        const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped pose;

        pose.header.stamp = now();
        pose.header.frame_id = "map";

        pose.pose.position.x = msg->x;
        pose.pose.position.y = msg->y;
        pose.pose.position.z = msg->z;

        pose.pose.orientation.w = 1.0;

        _path.header.stamp = now();
        _path.poses.push_back(pose);

        _path_pub->publish(_path);

        //
        // map -> base_link
        //

        geometry_msgs::msg::TransformStamped base_tf;

        base_tf.header.stamp = now();
        base_tf.header.frame_id = "map";
        base_tf.child_frame_id = "base_link";

        base_tf.transform.translation.x = msg->x;
        base_tf.transform.translation.y = msg->y;
        base_tf.transform.translation.z = msg->z;

        base_tf.transform.rotation.w = _attitude.q[0];
        base_tf.transform.rotation.x = _attitude.q[1];
        base_tf.transform.rotation.y = _attitude.q[2];
        base_tf.transform.rotation.z = _attitude.q[3];

        _tf_broadcaster->sendTransform(
            base_tf);

        //
        // base_link -> camera_link
        //

        if (!_have_base_pose ||
            !_have_camera_pose)
        {
            return;
        }

        Eigen::Quaterniond q_world_base(
            _base_pose.orientation().w(),
            _base_pose.orientation().x(),
            _base_pose.orientation().y(),
            _base_pose.orientation().z());

        Eigen::Quaterniond q_world_camera(
            _camera_pose.orientation().w(),
            _camera_pose.orientation().x(),
            _camera_pose.orientation().y(),
            _camera_pose.orientation().z());

        Eigen::Vector3d p_world_base(
            _base_pose.position().x(),
            _base_pose.position().y(),
            _base_pose.position().z());

        Eigen::Vector3d p_world_camera(
            _camera_pose.position().x(),
            _camera_pose.position().y(),
            _camera_pose.position().z());

        Eigen::Isometry3d T_world_base =
            Eigen::Isometry3d::Identity();

        T_world_base.linear() =
            q_world_base.toRotationMatrix();

        T_world_base.translation() =
            p_world_base;

        Eigen::Isometry3d T_world_camera =
            Eigen::Isometry3d::Identity();

        T_world_camera.linear() =
            q_world_camera.toRotationMatrix();

        T_world_camera.translation() =
            p_world_camera;

        Eigen::Isometry3d T_base_camera =
            T_world_base.inverse()
            *
            T_world_camera;

        Eigen::Quaterniond q_base_camera(
            T_base_camera.rotation());

        geometry_msgs::msg::TransformStamped camera_tf;

        camera_tf.header.stamp = now();
        camera_tf.header.frame_id = "base_link";
        camera_tf.child_frame_id = "camera_link";

        camera_tf.transform.translation.x =
            T_base_camera.translation().x();

        camera_tf.transform.translation.y =
            T_base_camera.translation().y();

        camera_tf.transform.translation.z =
            T_base_camera.translation().z();

        camera_tf.transform.rotation.x =
            q_base_camera.x();

        camera_tf.transform.rotation.y =
            q_base_camera.y();

        camera_tf.transform.rotation.z =
            q_base_camera.z();

        camera_tf.transform.rotation.w =
            q_base_camera.w();

        _tf_broadcaster->sendTransform(
            camera_tf);

        //
        // camera_link -> camera_optical_frame
        //

        geometry_msgs::msg::TransformStamped optical_tf;

        optical_tf.header.stamp = now();
        optical_tf.header.frame_id = "camera_link";
        optical_tf.child_frame_id = "camera_optical_frame";

        optical_tf.transform.translation.x = 0.0;
        optical_tf.transform.translation.y = 0.0;
        optical_tf.transform.translation.z = 0.0;

        tf2::Quaternion q_optical;

        q_optical.setRPY(
            -M_PI_2,
            0.0,
            -M_PI_2
        );

        optical_tf.transform.rotation.x =
            q_optical.x();

        optical_tf.transform.rotation.y =
            q_optical.y();

        optical_tf.transform.rotation.z =
            q_optical.z();

        optical_tf.transform.rotation.w =
            q_optical.w();

        _tf_broadcaster->sendTransform(
            optical_tf);
    }

    nav_msgs::msg::Path _path;

    rclcpp::Publisher<
        nav_msgs::msg::Path>::SharedPtr
        _path_pub;

    rclcpp::Subscription<
        px4_msgs::msg::VehicleLocalPosition>::SharedPtr
        _local_position_sub;

    rclcpp::Subscription<
        px4_msgs::msg::VehicleAttitude>::SharedPtr
        _attitude_sub;

    px4_msgs::msg::VehicleAttitude
        _attitude;

    std::unique_ptr<
        tf2_ros::TransformBroadcaster>
        _tf_broadcaster;

    gz::transport::Node
        _gz_node;

    gz::msgs::Pose
        _base_pose;

    gz::msgs::Pose
        _camera_pose;

    bool _have_base_pose{false};

    bool _have_camera_pose{false};
};