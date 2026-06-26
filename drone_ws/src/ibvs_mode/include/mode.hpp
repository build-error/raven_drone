#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/attitude.hpp>

#include <ibvs_interfaces/msg/ibvs_velocity.hpp>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals; // NOLINT

static const std::string kName = "IBVS Mode";

class IBVSMode : public px4_ros2::ModeBase
{
public:
    explicit IBVSMode(rclcpp::Node & node)
    : ModeBase(node, Settings{kName}.preventArming(false)),
      _node(node)
    {
        _trajectory_setpoint =
            std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

        _vehicle_attitude =
            std::make_shared<px4_ros2::OdometryAttitude>(*this);

        _ibvs_sub =
            node.create_subscription<
                ibvs_interfaces::msg::IBVSVelocity>(
                    "/ibvs/velocity_reference",
                    10,
                    [this](
                        const ibvs_interfaces::msg::IBVSVelocity::SharedPtr msg)
                    {
                        _last_ibvs = *msg;
                        _last_cmd_time = _node.get_clock()->now();
                    });
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

        if (!_last_ibvs.target_visible)
        {
            _trajectory_setpoint->update(
                Eigen::Vector3f::Zero(),
                std::nullopt,
                std::nullopt,
                0.0f);

            return;
        }

        const float yaw =
            _vehicle_attitude->yaw();

        const float vx_body =
            _last_ibvs.vx;

        const float vy_body =
            _last_ibvs.vy;

        const float vz_body =
            _last_ibvs.vz;

        const float vx_ned =
            vx_body * cosf(yaw)
            - vy_body * sinf(yaw);

        const float vy_ned =
            vx_body * sinf(yaw)
            + vy_body * cosf(yaw);

        const float vz_ned =
            vz_body;

        const Eigen::Vector3f velocity_sp{
            vx_ned,
            vy_ned,
            vz_ned
        };

        const float yaw_rate =
            _last_ibvs.wz;

        RCLCPP_INFO_THROTTLE(
            _node.get_logger(),
            *_node.get_clock(),
            1000,
            "IBVS: body=(%.2f %.2f %.2f) ned=(%.2f %.2f %.2f) yaw_rate=%.2f",
            vx_body,
            vy_body,
            vz_body,
            vx_ned,
            vy_ned,
            vz_ned,
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
    rclcpp::Node & _node;

    std::shared_ptr<px4_ros2::TrajectorySetpointType>
        _trajectory_setpoint;

    std::shared_ptr<px4_ros2::OdometryAttitude>
        _vehicle_attitude;

    ibvs_interfaces::msg::IBVSVelocity
        _last_ibvs;

    rclcpp::Subscription<
        ibvs_interfaces::msg::IBVSVelocity>::SharedPtr
        _ibvs_sub;

    rclcpp::Time
        _last_cmd_time;
};