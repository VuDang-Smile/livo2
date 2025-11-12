#ifndef QR_DETECTOR_LIB_HPP
#define QR_DETECTOR_LIB_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

namespace theta_driver {

class QRDetector : public rclcpp::Node {
public:
    QRDetector(const rclcpp::NodeOptions & options);
    virtual ~QRDetector();
    
private:
    void onInit();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_pub_;
    
    std::string camera_frame_;
    
    // QR Code detector
    cv::QRCodeDetector qr_detector_;
};

} // namespace theta_driver

#endif // QR_DETECTOR_LIB_HPP
