#ifndef CUBEMAP_CONVERTER_LIB_HPP
#define CUBEMAP_CONVERTER_LIB_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <string>

namespace theta_driver {

class CubemapConverter : public rclcpp::Node {
public:
    CubemapConverter(const rclcpp::NodeOptions & options);
    virtual ~CubemapConverter();
    
private:
    void onInit();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    // Convert equirectangular to cubemap faces
    void equirectangularToCubemap(const cv::Mat& equirect, 
                                  std::vector<cv::Mat>& faces,
                                  int face_width, 
                                  int face_height,
                                  double fov_degrees);
    
    // Convert a single cubemap face
    cv::Mat extractCubeFace(const cv::Mat& equirect, 
                           int face_idx,
                           int face_width,
                           int face_height,
                           double fov_degrees);
    
    // Convert cube coordinates to equirectangular coordinates
    void cubeToEquirect(double x, double y, int face,
                       double& theta, double& phi);
    
    // Combine 6 faces into a single image
    cv::Mat combineFaces(const std::vector<cv::Mat>& faces);
    
    // Rotate equirectangular image by specified degrees
    cv::Mat rotateEquirectangular(const cv::Mat& equirect, double degrees);
    
    // Publish combined image with description
    void publishCombinedImage(const cv::Mat& combined, 
                             const builtin_interfaces::msg::Time& timestamp,
                             const std::string& description);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr faces_pub_;
    
    std::string camera_frame_;
    int face_width_;
    int face_height_;
    double fov_degrees_;
    
    // Face order: front, right, back, left, top, bottom
    const std::vector<std::string> face_names_ = {
        "front", "right", "back", "left", "top", "bottom"
    };
};

} // namespace theta_driver

#endif // CUBEMAP_CONVERTER_LIB_HPP

