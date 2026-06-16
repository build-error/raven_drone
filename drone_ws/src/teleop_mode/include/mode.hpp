#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals; // NOLINT

static const std::string kName = "Teleop Mode";

class TeleopMode : public px4_ros2::ModeBase
{
    public:
    explicit TeleopMode(rclcpp::Node & node)
    : ModeBase(node, Settings{kName}.preventArming(false))
    {
        _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
        
        _cmd_vel_sub =
        node.create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                _last_cmd_vel = *msg;
            });
    }

    void onActivate() override {}

    void onDeactivate() override {}

    void updateSetpoint(float dt_s) override
    {
        (void)dt_s;

        const Eigen::Vector3f velocity_sp{
            static_cast<float>(_last_cmd_vel.linear.x),
            static_cast<float>(_last_cmd_vel.linear.y),
            static_cast<float>(_last_cmd_vel.linear.z)
        };

        const float yaw_rate =
            static_cast<float>(_last_cmd_vel.angular.z);

        RCLCPP_INFO_THROTTLE(
            node().get_logger(),
            *node().get_clock(),
            1000,
            "vx=%.2f vy=%.2f vz=%.2f yaw_rate=%.2f",
            velocity_sp.x(),
            velocity_sp.y(),
            velocity_sp.z(),
            yaw_rate
        );

        _trajectory_setpoint->update(
            velocity_sp,
            std::nullopt,
            std::nullopt,
            yaw_rate
        );
    }

    private:
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
    
    geometry_msgs::msg::Twist _last_cmd_vel;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
};