#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <termios.h>
#include <unistd.h>

class KeyboardTeleop : public rclcpp::Node
{
public:
    KeyboardTeleop()
    : Node("keyboard_teleop")
    {
        _publisher =
            create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10);

        configureTerminal();

        RCLCPP_INFO(get_logger(), " ");
        RCLCPP_INFO(get_logger(), "=== Keyboard Teleop ===");
        RCLCPP_INFO(get_logger(), "W/S : Forward / Backward");
        RCLCPP_INFO(get_logger(), "A/D : Left / Right");
        RCLCPP_INFO(get_logger(), "R/F : Up / Down");
        RCLCPP_INFO(get_logger(), "Q/E : Yaw Left / Right");
        RCLCPP_INFO(get_logger(), "SPACE : Stop");
        RCLCPP_INFO(get_logger(), "X : Exit");
        RCLCPP_INFO(get_logger(), " ");
    }

    ~KeyboardTeleop()
    {
        restoreTerminal();
    }

    void run()
    {
        constexpr float linear_speed = 1.0f;
        constexpr float vertical_speed = 1.0f;
        constexpr float yaw_speed = 1.0f;

        while (rclcpp::ok()) {
            char key = getKey();

            geometry_msgs::msg::Twist msg;

            switch (key) {
                case 'w':
                    msg.linear.x = linear_speed;
                    break;

                case 's':
                    msg.linear.x = -linear_speed;
                    break;

                case 'a':
                    msg.linear.y = linear_speed;
                    break;

                case 'd':
                    msg.linear.y = -linear_speed;
                    break;

                case 'r':
                    msg.linear.z = vertical_speed;
                    break;

                case 'f':
                    msg.linear.z = -vertical_speed;
                    break;

                case 'q':
                    msg.angular.z = yaw_speed;
                    break;

                case 'e':
                    msg.angular.z = -yaw_speed;
                    break;

                case ' ':
                    break;

                case 'x':
                    return;

                default:
                    continue;
            }

            _publisher->publish(msg);

            RCLCPP_INFO(
                get_logger(),
                "vx=%.2f vy=%.2f vz=%.2f yaw=%.2f",
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.z);
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;

    struct termios _original_terminal {};

    void configureTerminal()
    {
        tcgetattr(STDIN_FILENO, &_original_terminal);

        struct termios raw = _original_terminal;

        raw.c_lflag &= ~(ICANON | ECHO);

        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }

    void restoreTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &_original_terminal);
    }

    char getKey()
    {
        char c;
        read(STDIN_FILENO, &c, 1);
        return c;
    }
};