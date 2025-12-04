#ifndef ATAN_CONVERTER_LIB_HPP
#define ATAN_CONVERTER_LIB_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

namespace theta_driver {

class ATANConverter : public rclcpp::Node {
public:
    ATANConverter(const rclcpp::NodeOptions & options);
    virtual ~ATANConverter();
    
private:
    void onInit();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    // Convert equirectangular to ATAN projection
    cv::Mat equirectangularToATAN(const cv::Mat& equirect, 
                                   int output_width,
                                   int output_height);
    
    // Convert spherical coordinates to equirectangular pixel coordinates
    void sphericalToEquirect(double theta, double phi,
                             int equirect_width, int equirect_height,
                             int& px, int& py);
    
    // ATAN cam2world: convert pixel to 3D direction
    void cam2world(double u, double v, double& nx, double& ny, double& nz) const;
    
    // ATAN distortion functions
    double invrtrans(double r_distorted) const;  // Inverse radial distortion
    
    // Publish ATAN image
    void publishATANImage(const cv::Mat& atan_image, 
                         const builtin_interfaces::msg::Time& timestamp);
    
    // Publish camera info
    void publishCameraInfo(const builtin_interfaces::msg::Time& timestamp);
    
    // Calculate camera intrinsics from FOV
    void calculateCameraIntrinsics();
    
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
    
    // Camera intrinsics (normalized, like rpg_vikit)
    double fx_, fy_;           // Focal length (normalized 0-1)
    double fx_inv_, fy_inv_;   // Inverse focal length
    double cx_, cy_;           // Principal point (normalized 0-1)
    
    // ATAN distortion parameter (s = d0)
    double s_;                 // Distortion coefficient
    double s_inv_;            // 1.0 / s
    double tans_;              // 2.0 * tan(s / 2.0)
    double tans_inv_;          // 1.0 / tans_
    bool distortion_;          // true if s != 0
};

} // namespace theta_driver

#endif // ATAN_CONVERTER_LIB_HPP

