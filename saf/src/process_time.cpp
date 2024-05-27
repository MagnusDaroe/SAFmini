#include "rclcpp/rclcpp.hpp"

# Here a processtime node could be implemented in Cpp

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("process_time");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
