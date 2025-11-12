#include "theta_driver/perspective_converter_lib.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;
    auto node = std::make_shared<theta_driver::PerspectiveConverter>(options);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}

