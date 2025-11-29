#ifndef EQUIDISTANT_CONVERTER_LIB_HPP
#define EQUIDISTANT_CONVERTER_LIB_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

namespace theta_driver {

class EquidistantConverter : public rclcpp::Node {
public:
    EquidistantConverter(const rclcpp::NodeOptions & options);
    virtual ~EquidistantConverter();
    
private:
    void onInit();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    // Convert equirectangular to equidistant (fisheye)
    cv::Mat equirectangularToEquidistant(const cv::Mat& equirect, 
                                        int output_width,
                                        int output_height,
                                        double fx, double fy, double cx, double cy);
    
    // Convert spherical coordinates to equirectangular pixel coordinates
    void sphericalToEquirect(double theta, double phi,
                             int equirect_width, int equirect_height,
                             int& px, int& py);
    
    // Publish equidistant image
    void publishEquidistantImage(const cv::Mat& equidistant, 
                                const builtin_interfaces::msg::Time& timestamp);
    
    // Publish camera info
    void publishCameraInfo(const builtin_interfaces::msg::Time& timestamp);
    
    // Calculate camera intrinsics from FOV and image size
    void calculateCameraIntrinsics();
    
    // Parameter callback for realtime parameter updates
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr equidistant_pub_;
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
    
    // Distortion coefficients (k1, k2, k3, k4)
    double k1_, k2_, k3_, k4_;
};

} // namespace theta_driver

#endif // EQUIDISTANT_CONVERTER_LIB_HPP

