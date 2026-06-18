#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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

        _tf_broadcaster =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        _attitude_sub =
            create_subscription<
                px4_msgs::msg::VehicleAttitude>(
                    "/fmu/out/vehicle_attitude",
                    rclcpp::SensorDataQoS(),
                    [this](
                        const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
                    {
                        _attitude = *msg;

                        RCLCPP_INFO_THROTTLE(
                            get_logger(),
                            *get_clock(),
                            1000,
                            "q=[%.3f %.3f %.3f %.3f]",
                            msg->q[0],
                            msg->q[1],
                            msg->q[2],
                            msg->q[3]);
                    });

        RCLCPP_INFO(
            get_logger(),
            "Frame Visualizer Started");
    }

private:
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

        geometry_msgs::msg::TransformStamped tf;

        tf.header.stamp = now();
        tf.header.frame_id = "map";
        tf.child_frame_id = "base_link";

        tf.transform.translation.x = msg->x;
        tf.transform.translation.y = msg->y;
        tf.transform.translation.z = msg->z;

        tf.transform.rotation.w = _attitude.q[0];
        tf.transform.rotation.x = _attitude.q[1];
        tf.transform.rotation.y = _attitude.q[2];
        tf.transform.rotation.z = _attitude.q[3];

        _tf_broadcaster->sendTransform(tf);
    }

    nav_msgs::msg::Path _path;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
        _path_pub;

    rclcpp::Subscription<
        px4_msgs::msg::VehicleLocalPosition>::SharedPtr
        _local_position_sub;

    std::unique_ptr<tf2_ros::TransformBroadcaster>
    _tf_broadcaster;

    rclcpp::Subscription<
        px4_msgs::msg::VehicleAttitude>::SharedPtr
        _attitude_sub;

    px4_msgs::msg::VehicleAttitude
        _attitude;
};