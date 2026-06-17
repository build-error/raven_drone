#include <keyboard.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<KeyboardTeleop>();
    node->run();

    rclcpp::shutdown();

    return 0;
}