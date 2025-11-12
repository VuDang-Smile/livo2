#include "theta_driver/cubemap_converter_optimized.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;
    auto node = std::make_shared<theta_driver::CubemapConverterOptimized>(options);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
