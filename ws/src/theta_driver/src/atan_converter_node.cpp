#include <rclcpp/rclcpp.hpp>
#include "theta_driver/atan_converter_lib.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;
    auto node = std::make_shared<theta_driver::ATANConverter>(options);
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

