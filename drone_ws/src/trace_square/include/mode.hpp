#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals; // NOLINT

#define VELOCITY 1.0
#define DISTANCE 30.0

static const std::string kName = "Trace Square";

class TraceSquareMode : public px4_ros2::ModeBase
{
public:
    explicit TraceSquareMode(rclcpp::Node & node)
    : ModeBase(node, Settings{kName}.preventArming(false))
    , _node(node)
    {
        _trajectory_setpoint =
            std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    }

    void onActivate() override
    {
        _state = State::SIDE_1;
        _state_start_time = _node.get_clock()->now();

        RCLCPP_INFO(
            _node.get_logger(),
            "Trace Square Activated");
    }

    void onDeactivate() override
    {
        RCLCPP_INFO(
            _node.get_logger(),
            "Trace Square Deactivated");
    }

    void updateSetpoint(float dt_s) override
    {
        (void)dt_s;

        const double elapsed =
            (_node.get_clock()->now() - _state_start_time).seconds();

        float vx = 0.0f;
        float vy = 0.0f;

        switch (_state)
        {
            case State::SIDE_1:
            {
                vx = VELOCITY;

                if (elapsed > DISTANCE)
                {
                    RCLCPP_INFO(
                        _node.get_logger(),
                        "SIDE_1 -> SIDE_2");

                    _state = State::SIDE_2;
                    _state_start_time = _node.get_clock()->now();
                }
                break;
            }

            case State::SIDE_2:
            {
                vy = -VELOCITY;

                if (elapsed > DISTANCE)
                {
                    RCLCPP_INFO(
                        _node.get_logger(),
                        "SIDE_2 -> SIDE_3");

                    _state = State::SIDE_3;
                    _state_start_time = _node.get_clock()->now();
                }
                break;
            }

            case State::SIDE_3:
            {
                vx = -VELOCITY;

                if (elapsed > DISTANCE)
                {
                    RCLCPP_INFO(
                        _node.get_logger(),
                        "SIDE_3 -> SIDE_4");

                    _state = State::SIDE_4;
                    _state_start_time = _node.get_clock()->now();
                }
                break;
            }

            case State::SIDE_4:
            {
                vy = VELOCITY;

                if (elapsed > DISTANCE)
                {
                    RCLCPP_INFO(
                        _node.get_logger(),
                        "SIDE_4 -> FINISHED");

                    _state = State::FINISHED;
                }
                break;
            }

            case State::FINISHED:
            {
                _trajectory_setpoint->update(
                    Eigen::Vector3f::Zero(),
                    std::nullopt,
                    std::nullopt,
                    0.0f);

                RCLCPP_INFO(
                    _node.get_logger(),
                    "Trace Square Completed");

                completed(px4_ros2::Result::Success);
                return;
            }
        }

        const Eigen::Vector3f velocity_sp{
            vx,
            vy,
            0.0f
        };

        RCLCPP_INFO_THROTTLE(
            _node.get_logger(),
            *_node.get_clock(),
            1000,
            "State=%d vx=%.2f vy=%.2f",
            static_cast<int>(_state),
            vx,
            vy);

        _trajectory_setpoint->update(
            velocity_sp,
            std::nullopt,
            std::nullopt,
            0.0f);
    }

private:
    enum class State
    {
        SIDE_1,
        SIDE_2,
        SIDE_3,
        SIDE_4,
        FINISHED
    };

    rclcpp::Node & _node;

    State _state{State::SIDE_1};

    rclcpp::Time _state_start_time;

    std::shared_ptr<px4_ros2::TrajectorySetpointType>
        _trajectory_setpoint;
};