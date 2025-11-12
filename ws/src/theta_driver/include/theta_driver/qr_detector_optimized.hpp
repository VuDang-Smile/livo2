#ifndef QR_DETECTOR_OPTIMIZED_HPP
#define QR_DETECTOR_OPTIMIZED_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <vector>
#include <string>
#include <future>
#include <chrono>

namespace theta_driver {

struct QRCodeResult {
    std::string content;
    std::string face_name;
    int face_index;
    cv::Rect bounding_box;
    double confidence;
};

class QRDetectorOptimized : public rclcpp::Node {
public:
    QRDetectorOptimized(const rclcpp::NodeOptions & options);
    virtual ~QRDetectorOptimized();
    
private:
    void onInit();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    // Detect QR codes in individual faces (optimized version)
    std::vector<QRCodeResult> detectQRCodesInFacesOptimized(const std::vector<cv::Mat>& faces);
    
    // Detect QR codes in individual faces (sequential version)
    std::vector<QRCodeResult> detectQRCodesInFaces(const std::vector<cv::Mat>& faces);
    
    // Detect QR codes in a single face (optimized version)
    std::vector<QRCodeResult> detectQRCodesInFaceOptimized(const cv::Mat& face, 
                                                          const std::string& face_name,
                                                          int face_index);
    
    // Extract individual faces from combined image
    std::vector<cv::Mat> extractIndividualFaces(const cv::Mat& combined_image);
    
    // Publish QR code detection results
    void publishQRResults(const std::vector<QRCodeResult>& results);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_pub_;
    
    std::string camera_frame_;
    int face_width_;
    int face_height_;
    
    // QR Code detector
    cv::QRCodeDetector qr_detector_;
    
    // Face order: front, right, back, left, top, bottom
    const std::vector<std::string> face_names_ = {
        "front", "right", "back", "left", "top", "bottom"
    };
    
    // Detection parameters
    bool enable_qr_detection_;
    double min_confidence_;
    int detection_interval_;  // Process every N frames
    int frame_counter_;
    
    // Optimization parameters
    bool enable_preprocessing_;  // Enable histogram equalization and contrast enhancement
    bool enable_multithreading_;  // Enable parallel face processing
    int max_faces_per_thread_;  // Maximum faces per thread
    
    // Enhanced detection parameters
    bool enable_rotation_detection_;  // Enable rotation detection for QR codes
    int qr_scale_factor_;  // Scale factor for QR detection
    bool prioritize_top_bottom_;  // Prioritize top and bottom faces for detection
};

} // namespace theta_driver

#endif // QR_DETECTOR_OPTIMIZED_HPP
