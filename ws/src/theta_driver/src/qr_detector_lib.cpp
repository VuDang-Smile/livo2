#include "theta_driver/qr_detector_lib.hpp"
#include <zbar.h>

namespace theta_driver {

QRDetector::QRDetector(const rclcpp::NodeOptions & options)
    : Node("qr_detector", options) {
    RCLCPP_INFO(get_logger(), "Initializing QR Detector");
    onInit();
}

QRDetector::~QRDetector() {
    RCLCPP_INFO(get_logger(), "Shutting down QR Detector");
}

void QRDetector::onInit() {
    // Declare and get parameters
    declare_parameter<std::string>("camera_frame", "camera_link");
    get_parameter("camera_frame", camera_frame_);
    
    RCLCPP_INFO(get_logger(), "QR Detector initialized");
    
    // Create subscriber and publisher
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_faces", 10,
        std::bind(&QRDetector::imageCallback, this, std::placeholders::_1));
    
    qr_pub_ = this->create_publisher<std_msgs::msg::String>("detect_qrcode", 10);
}

void QRDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "📥 Received image: %dx%d, encoding: %s", 
                msg->width, msg->height, msg->encoding.c_str());
    
    try {
        // Convert ROS image to OpenCV Mat
        cv::Mat image;
        if (msg->encoding == "rgb8") {
            image = cv::Mat(msg->height, msg->width, CV_8UC3, 
                          const_cast<uint8_t*>(msg->data.data())).clone();
            RCLCPP_INFO(get_logger(), "✅ Converted RGB8 image to OpenCV Mat");
        } else if (msg->encoding == "bgr8") {
            cv::Mat temp(msg->height, msg->width, CV_8UC3, 
                        const_cast<uint8_t*>(msg->data.data()));
            cv::cvtColor(temp, image, cv::COLOR_BGR2RGB);
            RCLCPP_INFO(get_logger(), "✅ Converted BGR8 image to RGB OpenCV Mat");
        } else {
            RCLCPP_WARN(get_logger(), "❌ Unsupported encoding: %s", msg->encoding.c_str());
            return;
        }
        
        bool qr_found = false;
        
        // Try ZBar with grayscale image (most reliable)
        RCLCPP_INFO(get_logger(), "🔍 Testing QR detection with ZBar");
        
        try {
            zbar::ImageScanner scanner;
            scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
            
            // Convert to grayscale for ZBar
            cv::Mat gray_image;
            cv::cvtColor(image, gray_image, cv::COLOR_RGB2GRAY);
            RCLCPP_INFO(get_logger(), "🔍 Testing ZBar with grayscale: %dx%d", 
                       gray_image.cols, gray_image.rows);
            
            zbar::Image zbar_image(gray_image.cols, gray_image.rows, "Y800", 
                                  gray_image.data, gray_image.cols * gray_image.rows);
            
            int n = scanner.scan(zbar_image);
            RCLCPP_INFO(get_logger(), "🔍 ZBar scan result: %d symbols found", n);
            
            if (n > 0) {
                for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
                     symbol != zbar_image.symbol_end(); ++symbol) {
                    
                    std::string qr_content = symbol->get_data();
                    RCLCPP_INFO(get_logger(), "📝 QR content (ZBar): '%s' (length=%zu)", 
                               qr_content.c_str(), qr_content.length());
                    
                    if (!qr_content.empty()) {
                        std_msgs::msg::String qr_msg;
                        qr_msg.data = qr_content;
                        qr_pub_->publish(qr_msg);
                        
                        RCLCPP_INFO(get_logger(), "✅ QR Code detected and published (ZBar): %s", 
                                   qr_content.c_str());
                        qr_found = true;
                        break;
                    }
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "⚠️ ZBar error: %s", e.what());
        }
        
        // Fallback to OpenCV if ZBar fails
        if (!qr_found) {
            RCLCPP_INFO(get_logger(), "🔍 ZBar failed, trying OpenCV fallback");
            
            // Convert to grayscale for OpenCV
            cv::Mat gray_image;
            cv::cvtColor(image, gray_image, cv::COLOR_RGB2GRAY);
            RCLCPP_INFO(get_logger(), "🔍 Testing OpenCV with grayscale: %dx%d", 
                       gray_image.cols, gray_image.rows);
            
            // Detect QR code with OpenCV
            std::vector<cv::Point2f> points;
            std::vector<cv::Mat> straight_qrcode;
            std::vector<std::string> decoded_infos;
            
            bool detected = qr_detector_.detectAndDecodeMulti(gray_image, decoded_infos, points, straight_qrcode);
            
            if (!detected) {
                std::string single_result;
                std::vector<cv::Point2f> single_points;
                single_result = qr_detector_.detectAndDecode(gray_image, single_points);
                if (!single_result.empty()) {
                    detected = true;
                    decoded_infos.clear();
                    decoded_infos.push_back(single_result);
                    points = single_points;
                }
            }
            
            RCLCPP_INFO(get_logger(), "🔍 OpenCV result: detected=%d, decoded_infos.size()=%zu", 
                       detected, decoded_infos.size());
            
            if (detected && !decoded_infos.empty()) {
                std::string qr_content = decoded_infos[0];
                if (!qr_content.empty()) {
                    std_msgs::msg::String qr_msg;
                    qr_msg.data = qr_content;
                    qr_pub_->publish(qr_msg);
                    
                    RCLCPP_INFO(get_logger(), "✅ QR Code detected and published (OpenCV): %s", 
                               qr_content.c_str());
                    qr_found = true;
                }
            }
        }
        
        if (!qr_found) {
            RCLCPP_INFO(get_logger(), "❌ No QR code detected with ZBar or OpenCV");
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "💥 Error in QR detection: %s", e.what());
    }
}

} // namespace theta_driver
