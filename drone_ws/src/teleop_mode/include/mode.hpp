#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/attitude.hpp>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals; // NOLINT

static const std::string kName = "Teleop Mode";

class TeleopMode : public px4_ros2::ModeBase
{
    public:
    explicit TeleopMode(rclcpp::Node & node)
    : ModeBase(node, Settings{kName}.preventArming(false)), _node(node)
    {
        _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
        
        _cmd_vel_sub =
            node.create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10,
                [this](const geometry_msgs::msg::Twist::SharedPtr msg)
                {
                    _last_cmd_vel = *msg;
                    _last_cmd_time = _node.get_clock()->now();
                });

        _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
    }

    void onActivate() override
    {
        _last_cmd_time = _node.get_clock()->now();
    }

    void onDeactivate() override {}

    void updateSetpoint(float dt_s) override
    {
        (void)dt_s;

        if ((_node.get_clock()->now() - _last_cmd_time).seconds() > 0.5)
        {
            _trajectory_setpoint->update(
                Eigen::Vector3f::Zero(),
                std::nullopt,
                std::nullopt,
                0.0f);

            return;
        }

        const float yaw = _vehicle_attitude->yaw();

        const float vx_body =
            static_cast<float>(_last_cmd_vel.linear.x);

        const float vy_body =
            static_cast<float>(_last_cmd_vel.linear.y);

        const float vz_body =
            static_cast<float>(_last_cmd_vel.linear.z);

        const float vx_ned =
            vx_body * cosf(yaw)
            - vy_body * sinf(yaw);

        const float vy_ned =
            vx_body * sinf(yaw)
            + vy_body * cosf(yaw);

        const float vz_ned =
            -vz_body;

        const Eigen::Vector3f velocity_sp{
            vx_ned,
            vy_ned,
            vz_ned
        };

        const float yaw_rate =
            -static_cast<float>(_last_cmd_vel.angular.z);

        RCLCPP_INFO_THROTTLE(
            _node.get_logger(),
            *_node.get_clock(),
            1000,
            "yaw=%.2f body=(%.2f %.2f %.2f) ned=(%.2f %.2f %.2f)",
            yaw,
            vx_body,
            vy_body,
            vz_body,
            vx_ned,
            vy_ned,
            vz_ned
        );

        _trajectory_setpoint->update(
            velocity_sp,
            std::nullopt,
            std::nullopt,
            yaw_rate
        );
    }

    private:
    rclcpp::Node& _node;

    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

    geometry_msgs::msg::Twist _last_cmd_vel;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;

    std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
    rclcpp::Time _last_cmd_time;
};