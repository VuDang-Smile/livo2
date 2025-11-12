#ifndef CUBEMAP_CONVERTER_OPTIMIZED_HPP
#define CUBEMAP_CONVERTER_OPTIMIZED_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <string>
#include <thread>
#include <future>
#include <chrono>

namespace theta_driver {

struct LookupEntry {
    int u;  // Fixed-point u coordinate (16-bit fractional part)
    int v;  // Fixed-point v coordinate (16-bit fractional part)
};

class CubemapConverterOptimized : public rclcpp::Node {
public:
    CubemapConverterOptimized(const rclcpp::NodeOptions & options);
    virtual ~CubemapConverterOptimized();
    
private:
    void onInit();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    // Pre-compute lookup tables for performance
    void precomputeLookupTables();
    
    // Optimized conversion methods
    void equirectangularToCubemapOptimized(const cv::Mat& equirect, 
                                          std::vector<cv::Mat>& faces);
    
    void extractCubeFaceOptimized(const cv::Mat& equirect, 
                                 cv::Mat& face,
                                 int face_idx,
                                 int equirect_width,
                                 int equirect_height,
                                 int face_width,
                                 int face_height);
    
    // Convert equirectangular to cubemap faces (original method for fallback)
    void equirectangularToCubemap(const cv::Mat& equirect, 
                                  std::vector<cv::Mat>& faces,
                                  int face_width, 
                                  int face_height,
                                  double fov_degrees);
    
    // Convert a single cubemap face (original method for fallback)
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
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr faces_pub_;
    
    std::string camera_frame_;
    int face_width_;
    int face_height_;
    double fov_degrees_;
    bool use_lookup_table_;
    int num_threads_;
    
    // Pre-computed lookup tables for each face
    std::vector<std::vector<LookupEntry>> lookup_tables_;
    
    // Face order: front, right, back, left, top, bottom
    const std::vector<std::string> face_names_ = {
        "front", "right", "back", "left", "top", "bottom"
    };
};

} // namespace theta_driver

#endif // CUBEMAP_CONVERTER_OPTIMIZED_HPP
