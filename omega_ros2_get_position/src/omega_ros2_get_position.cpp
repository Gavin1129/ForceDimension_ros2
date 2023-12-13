#include "rclcpp/rclcpp.hpp"
#include "omega_ros2_get_position/HapticDevice.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create a ROS2 node
    auto node = std::make_shared<rclcpp::Node>("omega_ros2_get_position");

    // Create an instance of HapticDevice with the node
    HapticDevice haptic_dev(node, 1000, true);  // Assuming these parameters are appropriate

    // Start the HapticDevice operation
    haptic_dev.Start();

    // Spin the node
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}


