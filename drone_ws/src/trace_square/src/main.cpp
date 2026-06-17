#include "rclcpp/rclcpp.hpp"

#include <mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

using MyNodeWithMode = px4_ros2::NodeWithMode<TraceSquareMode>;

static const std::string kNodeName = "trace_square";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<MyNodeWithMode>(
            kNodeName,
            kEnableDebugOutput));

    rclcpp::shutdown();

    return 0;
}