#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals; // NOLINT

static const std::string kName = "Teleop Mode";

class TeleopMode : public px4_ros2::ModeBase
{
    public:
    explicit TeleopMode(rclcpp::Node & node)
    : ModeBase(node, Settings{kName}.preventArming(false))
    {
        _trajectory_setpoint =
        std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    }

    void onActivate() override {}

    void onDeactivate() override {}

    void updateSetpoint(float dt_s) override
    {
        (void)dt_s;

        RCLCPP_INFO_THROTTLE(
            node().get_logger(),
            *node().get_clock(),
            1000,
            "updateSetpoint running"
        );

        const Eigen::Vector3f velocity_sp{
            0.0f,
            0.0f,
            -1.0f
        };

        _trajectory_setpoint->update(
            velocity_sp,
            std::nullopt,
            std::nullopt,
            std::nullopt
        );
    }

    private:
    std::shared_ptr<px4_ros2::TrajectorySetpointType>
        _trajectory_setpoint;
};