#ifndef PERSPECTIVE_CONVERTER_LIB_HPP
#define PERSPECTIVE_CONVERTER_LIB_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

namespace theta_driver {

class PerspectiveConverter : public rclcpp::Node {
public:
    PerspectiveConverter(const rclcpp::NodeOptions & options);
    virtual ~PerspectiveConverter();
    
private:
    void onInit();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    // Convert equirectangular to perspective (front view)
    cv::Mat equirectangularToPerspective(const cv::Mat& equirect, 
                                        int output_width,
                                        int output_height,
                                        double fov_degrees);
    
    // Convert spherical coordinates to equirectangular pixel coordinates
    void sphericalToEquirect(double theta, double phi,
                             int equirect_width, int equirect_height,
                             int& px, int& py);
    
    // Publish perspective image
    void publishPerspectiveImage(const cv::Mat& perspective, 
                                const builtin_interfaces::msg::Time& timestamp);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr perspective_pub_;
    
    std::string camera_frame_;
    std::string input_topic_;
    std::string output_topic_;
    int output_width_;
    int output_height_;
    double fov_degrees_;
};

} // namespace theta_driver

#endif // PERSPECTIVE_CONVERTER_LIB_HPP

