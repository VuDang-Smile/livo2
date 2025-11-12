#include "theta_driver/qr_detector_lib.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;
    auto node = std::make_shared<theta_driver::QRDetector>(options);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
