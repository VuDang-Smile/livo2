#include "theta_driver/camera_info_publisher_lib.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    // Init rclcpp
    rclcpp::init(argc, argv);
    // Create executor
    rclcpp::executors::SingleThreadedExecutor exec;
    // Create empty options
    rclcpp::NodeOptions options;
    // Create the camera_info_publisher
    auto camera_info_publisher = std::make_shared<theta_driver::CameraInfoPublisher>(options);
    // Add the node to the executor
    exec.add_node(camera_info_publisher);
    // Spin the node
    exec.spin();
    // Shutdown rclcpp
    rclcpp::shutdown();
    return 0;
}
