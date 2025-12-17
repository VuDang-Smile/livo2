#ifndef THETA_DRIVER_CAMERA_INFO_PUBLISHER_LIB_HPP
#define THETA_DRIVER_CAMERA_INFO_PUBLISHER_LIB_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>

namespace theta_driver {

class CameraInfoPublisher : public rclcpp::Node {
public:
    CameraInfoPublisher(const rclcpp::NodeOptions & options);
    virtual ~CameraInfoPublisher();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void publishCameraInfo(const sensor_msgs::msg::Image::SharedPtr image_msg);
    sensor_msgs::msg::CameraInfo createDefaultCameraInfo(uint32_t width, uint32_t height);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    
    std::string image_topic_;
    std::string camera_info_topic_;
    std::string camera_frame_;
    std::string camera_name_;
    
    // Camera parameters (can be loaded from calibration file or set via parameters)
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double k1_, k2_, p1_, p2_, k3_;
    
    bool use_calibration_params_;
};

} // namespace theta_driver

#endif // THETA_DRIVER_CAMERA_INFO_PUBLISHER_LIB_HPP



