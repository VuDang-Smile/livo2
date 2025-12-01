#include "theta_driver/polynomial_converter_lib.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;
    auto node = std::make_shared<theta_driver::PolynomialConverter>(options);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}

