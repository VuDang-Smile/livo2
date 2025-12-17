#include "theta_driver/camera_info_publisher_lib.hpp"
#include <sensor_msgs/msg/camera_info.hpp>

namespace theta_driver {

CameraInfoPublisher::CameraInfoPublisher(const rclcpp::NodeOptions & options)
    : Node("camera_info_publisher", options)
{
    // Declare parameters
    declare_parameter<std::string>("image_topic", "/image_raw");
    declare_parameter<std::string>("camera_info_topic", "/camera_info");
    declare_parameter<std::string>("camera_frame", "camera_link");
    declare_parameter<std::string>("camera_name", "theta_camera");
    
    // Camera intrinsic parameters (for calibration)
    declare_parameter<double>("fx", 0.0);
    declare_parameter<double>("fy", 0.0);
    declare_parameter<double>("cx", 0.0);
    declare_parameter<double>("cy", 0.0);
    
    // Distortion parameters
    declare_parameter<double>("k1", 0.0);
    declare_parameter<double>("k2", 0.0);
    declare_parameter<double>("p1", 0.0);
    declare_parameter<double>("p2", 0.0);
    declare_parameter<double>("k3", 0.0);
    
    declare_parameter<bool>("use_calibration_params", false);
    
    // Get parameters
    get_parameter("image_topic", image_topic_);
    get_parameter("camera_info_topic", camera_info_topic_);
    get_parameter("camera_frame", camera_frame_);
    get_parameter("camera_name", camera_name_);
    
    get_parameter("fx", fx_);
    get_parameter("fy", fy_);
    get_parameter("cx", cx_);
    get_parameter("cy", cy_);
    
    get_parameter("k1", k1_);
    get_parameter("k2", k2_);
    get_parameter("p1", p1_);
    get_parameter("p2", p2_);
    get_parameter("k3", k3_);
    
    get_parameter("use_calibration_params", use_calibration_params_);
    
    // Create subscriber for image topic
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_,
        10,
        std::bind(&CameraInfoPublisher::imageCallback, this, std::placeholders::_1)
    );
    
    // Create publisher for camera_info
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_,
        10
    );
    
    RCLCPP_INFO(get_logger(), "CameraInfoPublisher initialized");
    RCLCPP_INFO(get_logger(), "  Subscribing to: %s", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Publishing to: %s", camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Camera frame: %s", camera_frame_.c_str());
    
    if (use_calibration_params_) {
        RCLCPP_INFO(get_logger(), "Using calibration parameters:");
        RCLCPP_INFO(get_logger(), "  fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx_, fy_, cx_, cy_);
        RCLCPP_INFO(get_logger(), "  k1=%.6f, k2=%.6f, p1=%.6f, p2=%.6f, k3=%.6f", 
                   k1_, k2_, p1_, p2_, k3_);
    } else {
        RCLCPP_INFO(get_logger(), "Using default (uncalibrated) camera info for calibration");
    }
}

CameraInfoPublisher::~CameraInfoPublisher()
{
    RCLCPP_INFO(get_logger(), "CameraInfoPublisher shutting down");
}

void CameraInfoPublisher::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    publishCameraInfo(msg);
}

void CameraInfoPublisher::publishCameraInfo(const sensor_msgs::msg::Image::SharedPtr image_msg)
{
    sensor_msgs::msg::CameraInfo camera_info;
    
    if (use_calibration_params_ && fx_ > 0.0 && fy_ > 0.0) {
        // Use provided calibration parameters
        camera_info.header = image_msg->header;
        camera_info.header.frame_id = camera_frame_;
        camera_info.width = image_msg->width;
        camera_info.height = image_msg->height;
        camera_info.distortion_model = "plumb_bob";
        
        // Intrinsic camera matrix (3x3 row-major)
        camera_info.k[0] = fx_;  // fx
        camera_info.k[1] = 0.0;
        camera_info.k[2] = cx_;  // cx
        camera_info.k[3] = 0.0;
        camera_info.k[4] = fy_;  // fy
        camera_info.k[5] = cy_;  // cy
        camera_info.k[6] = 0.0;
        camera_info.k[7] = 0.0;
        camera_info.k[8] = 1.0;
        
        // Distortion parameters (k1, k2, p1, p2, k3)
        camera_info.d.resize(5);
        camera_info.d[0] = k1_;
        camera_info.d[1] = k2_;
        camera_info.d[2] = p1_;
        camera_info.d[3] = p2_;
        camera_info.d[4] = k3_;
        
        // Rectification matrix (identity for monocular)
        camera_info.r[0] = 1.0;
        camera_info.r[1] = 0.0;
        camera_info.r[2] = 0.0;
        camera_info.r[3] = 0.0;
        camera_info.r[4] = 1.0;
        camera_info.r[5] = 0.0;
        camera_info.r[6] = 0.0;
        camera_info.r[7] = 0.0;
        camera_info.r[8] = 1.0;
        
        // Projection matrix (same as K for monocular)
        camera_info.p[0] = fx_;
        camera_info.p[1] = 0.0;
        camera_info.p[2] = cx_;
        camera_info.p[3] = 0.0;
        camera_info.p[4] = 0.0;
        camera_info.p[5] = fy_;
        camera_info.p[6] = cy_;
        camera_info.p[7] = 0.0;
        camera_info.p[8] = 0.0;
        camera_info.p[9] = 0.0;
        camera_info.p[10] = 1.0;
        camera_info.p[11] = 0.0;
    } else {
        // Create default camera info for calibration (uncalibrated)
        camera_info = createDefaultCameraInfo(image_msg->width, image_msg->height);
        camera_info.header = image_msg->header;
        camera_info.header.frame_id = camera_frame_;
    }
    
    camera_info_pub_->publish(camera_info);
}

sensor_msgs::msg::CameraInfo CameraInfoPublisher::createDefaultCameraInfo(uint32_t width, uint32_t height)
{
    sensor_msgs::msg::CameraInfo camera_info;
    
    camera_info.width = width;
    camera_info.height = height;
    camera_info.distortion_model = "plumb_bob";
    
    // Default intrinsic matrix (will be calibrated)
    // Use image center as principal point and reasonable default focal length
    double default_fx = static_cast<double>(width) * 0.8;  // Reasonable default
    double default_fy = static_cast<double>(height) * 0.8;
    double default_cx = static_cast<double>(width) / 2.0;
    double default_cy = static_cast<double>(height) / 2.0;
    
    // Intrinsic camera matrix (3x3 row-major)
    camera_info.k[0] = default_fx;  // fx
    camera_info.k[1] = 0.0;
    camera_info.k[2] = default_cx;  // cx
    camera_info.k[3] = 0.0;
    camera_info.k[4] = default_fy;  // fy
    camera_info.k[5] = default_cy;  // cy
    camera_info.k[6] = 0.0;
    camera_info.k[7] = 0.0;
    camera_info.k[8] = 1.0;
    
    // Distortion parameters (zero for uncalibrated, will be set during calibration)
    camera_info.d.resize(5);
    camera_info.d[0] = 0.0;  // k1
    camera_info.d[1] = 0.0;  // k2
    camera_info.d[2] = 0.0;  // p1
    camera_info.d[3] = 0.0;  // p2
    camera_info.d[4] = 0.0;  // k3
    
    // Rectification matrix (identity for monocular)
    camera_info.r[0] = 1.0;
    camera_info.r[1] = 0.0;
    camera_info.r[2] = 0.0;
    camera_info.r[3] = 0.0;
    camera_info.r[4] = 1.0;
    camera_info.r[5] = 0.0;
    camera_info.r[6] = 0.0;
    camera_info.r[7] = 0.0;
    camera_info.r[8] = 1.0;
    
    // Projection matrix (same as K for monocular)
    camera_info.p[0] = default_fx;
    camera_info.p[1] = 0.0;
    camera_info.p[2] = default_cx;
    camera_info.p[3] = 0.0;
    camera_info.p[4] = 0.0;
    camera_info.p[5] = default_fy;
    camera_info.p[6] = default_cy;
    camera_info.p[7] = 0.0;
    camera_info.p[8] = 0.0;
    camera_info.p[9] = 0.0;
    camera_info.p[10] = 1.0;
    camera_info.p[11] = 0.0;
    
    return camera_info;
}

} // namespace theta_driver



