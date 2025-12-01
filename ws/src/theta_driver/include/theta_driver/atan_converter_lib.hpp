#ifndef ATAN_CONVERTER_LIB_HPP
#define ATAN_CONVERTER_LIB_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

namespace theta_driver {

class ATANConverter : public rclcpp::Node {
public:
    ATANConverter(const rclcpp::NodeOptions & options);
    virtual ~ATANConverter();
    
private:
    void onInit();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    // Convert equirectangular to ATAN
    cv::Mat equirectangularToATAN(const cv::Mat& equirect, 
                                  int output_width,
                                  int output_height,
                                  double fx, double fy, double cx, double cy);
    
    // Convert spherical coordinates to equirectangular pixel coordinates
    void sphericalToEquirect(double theta, double phi,
                             int equirect_width, int equirect_height,
                             int& px, int& py);
    
    // Publish ATAN image
    void publishATANImage(const cv::Mat& atan_image, 
                         const builtin_interfaces::msg::Time& timestamp);
    
    // Publish camera info
    void publishCameraInfo(const builtin_interfaces::msg::Time& timestamp);
    
    // Calculate camera intrinsics from FOV and image size
    void calculateCameraIntrinsics();
    
    // ATAN distortion functions
    double rtrans_factor(double r) const;  // Returns ratio of distorted / undistorted radius
    double invrtrans(double r) const;      // Inverse radial distortion
    
    // Parameter callback for realtime parameter updates
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr atan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    std::string camera_frame_;
    std::string input_topic_;
    std::string output_topic_;
    std::string camera_info_topic_;
    int output_width_;
    int output_height_;
    double fov_degrees_;
    
    // Camera intrinsics
    double fx_, fy_, cx_, cy_;
    
    // ATAN distortion coefficient (s or d0)
    double d0_;
    double s_;           // s = d0
    double s_inv_;       // 1.0 / s
    double tans_;        // 2.0 * tan(s / 2.0)
    double tans_inv_;    // 1.0 / tans_
    bool distortion_;    // true if d0 != 0
};

} // namespace theta_driver

#endif // ATAN_CONVERTER_LIB_HPP

