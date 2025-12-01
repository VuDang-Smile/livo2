#ifndef POLYNOMIAL_CONVERTER_LIB_HPP
#define POLYNOMIAL_CONVERTER_LIB_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

namespace theta_driver {

class PolynomialConverter : public rclcpp::Node {
public:
    PolynomialConverter(const rclcpp::NodeOptions & options);
    virtual ~PolynomialConverter();
    
private:
    void onInit();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    // Convert equirectangular to polynomial
    cv::Mat equirectangularToPolynomial(const cv::Mat& equirect, 
                                        int output_width,
                                        int output_height,
                                        double fx, double fy, double cx, double cy, double skew);
    
    // Convert spherical coordinates to equirectangular pixel coordinates
    void sphericalToEquirect(double theta, double phi,
                             int equirect_width, int equirect_height,
                             int& px, int& py);
    
    // Publish polynomial image
    void publishPolynomialImage(const cv::Mat& polynomial, 
                                const builtin_interfaces::msg::Time& timestamp);
    
    // Publish camera info
    void publishCameraInfo(const builtin_interfaces::msg::Time& timestamp);
    
    // Calculate camera intrinsics from FOV and image size
    void calculateCameraIntrinsics();
    
    // Parameter callback for realtime parameter updates
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr polynomial_pub_;
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
    double skew_;
    
    // Distortion coefficients (k2, k3, k4, k5, k6, k7)
    double k2_, k3_, k4_, k5_, k6_, k7_;
    
    // Rotation offset to align inner circle with outer border (degrees)
    double rotation_offset_degrees_;
};

} // namespace theta_driver

#endif // POLYNOMIAL_CONVERTER_LIB_HPP

