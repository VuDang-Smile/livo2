#include "theta_driver/perspective_converter_lib.hpp"
#include <cmath>

namespace theta_driver {

PerspectiveConverter::PerspectiveConverter(const rclcpp::NodeOptions & options)
    : Node("perspective_converter", options) {
    RCLCPP_INFO(get_logger(), "Initializing Perspective Converter");
    onInit();
}

PerspectiveConverter::~PerspectiveConverter() {
    RCLCPP_INFO(get_logger(), "Shutting down Perspective Converter");
}

void PerspectiveConverter::onInit() {
    // Declare and get parameters
    declare_parameter<std::string>("camera_frame", "camera_link");
    get_parameter("camera_frame", camera_frame_);
    
    declare_parameter<std::string>("input_topic", "image_raw");
    get_parameter("input_topic", input_topic_);
    
    declare_parameter<std::string>("output_topic", "image_perspective");
    get_parameter("output_topic", output_topic_);
    
    declare_parameter<int>("output_width", 640);
    get_parameter("output_width", output_width_);
    
    declare_parameter<int>("output_height", 480);
    get_parameter("output_height", output_height_);
    
    declare_parameter<double>("fov_degrees", 75.0);
    get_parameter("fov_degrees", fov_degrees_);
    
    declare_parameter<std::string>("camera_info_topic", "camera_info");
    get_parameter("camera_info_topic", camera_info_topic_);
    
    declare_parameter<std::string>("distortion_model", "plumb_bob");
    get_parameter("distortion_model", distortion_model_);
    
    RCLCPP_INFO(get_logger(), "Perspective converter parameters:");
    RCLCPP_INFO(get_logger(), "  Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Camera info topic: %s", camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Output size: %dx%d", output_width_, output_height_);
    RCLCPP_INFO(get_logger(), "  FOV: %.1f degrees", fov_degrees_);
    
    // Calculate camera intrinsics
    calculateCameraIntrinsics();
    
    // Create subscriber and publisher
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_topic_, 10,
        std::bind(&PerspectiveConverter::imageCallback, this, std::placeholders::_1));
    
    perspective_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10);
    
    RCLCPP_INFO(get_logger(), "Perspective Converter initialized");
    RCLCPP_INFO(get_logger(), "  Camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                fx_, fy_, cx_, cy_);
}

void PerspectiveConverter::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // Convert ROS image to OpenCV Mat
        cv::Mat equirect;
        if (msg->encoding == "rgb8") {
            equirect = cv::Mat(msg->height, msg->width, CV_8UC3, 
                             const_cast<uint8_t*>(msg->data.data())).clone();
        } else if (msg->encoding == "bgr8") {
            cv::Mat temp(msg->height, msg->width, CV_8UC3, 
                        const_cast<uint8_t*>(msg->data.data()));
            cv::cvtColor(temp, equirect, cv::COLOR_BGR2RGB);
        } else {
            RCLCPP_WARN(get_logger(), "Unsupported encoding: %s", msg->encoding.c_str());
            return;
        }
        
        // Convert equirectangular to perspective
        cv::Mat perspective = equirectangularToPerspective(
            equirect, output_width_, output_height_, fov_degrees_);
        
        // Publish perspective image
        publishPerspectiveImage(perspective, msg->header.stamp);
        
        // Publish camera info
        publishCameraInfo(msg->header.stamp);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in image callback: %s", e.what());
    }
}

void PerspectiveConverter::sphericalToEquirect(double theta, double phi,
                                               int equirect_width, int equirect_height,
                                               int& px, int& py) {
    // theta: longitude [-pi, pi] -> u: [0, 1]
    // phi: latitude [-pi/2, pi/2] -> v: [0, 1]
    double u = (theta / M_PI + 1.0) / 2.0;
    double v = (phi / M_PI + 0.5);
    
    px = static_cast<int>(u * equirect_width) % equirect_width;
    py = std::min(std::max(static_cast<int>(v * equirect_height), 0), 
                 equirect_height - 1);
}

cv::Mat PerspectiveConverter::equirectangularToPerspective(const cv::Mat& equirect, 
                                                           int output_width,
                                                           int output_height,
                                                           double fov_degrees) {
    cv::Mat perspective(output_height, output_width, CV_8UC3);
    
    int equirect_width = equirect.cols;
    int equirect_height = equirect.rows;
    
    // Convert FOV from degrees to radians
    double fov_rad = fov_degrees * M_PI / 180.0;
    double half_fov = tan(fov_rad / 2.0);
    
    // Camera is looking forward (theta = 0, phi = 0)
    // For perspective projection, we map pixel coordinates to 3D direction vectors
    // then convert to spherical coordinates
    
    for (int j = 0; j < output_height; j++) {
        for (int i = 0; i < output_width; i++) {
            // Convert pixel coordinates to normalized coordinates [-1, 1]
            // Center at (0, 0) with aspect ratio correction
            double aspect_ratio = static_cast<double>(output_width) / output_height;
            double x = (2.0 * i / output_width - 1.0) * aspect_ratio * half_fov;
            double y = (2.0 * j / output_height - 1.0) * half_fov;
            
            // For front-facing perspective view:
            // x, y are in the image plane, z = 1 (looking forward)
            // Normalize to get direction vector
            double norm = sqrt(x * x + y * y + 1.0);
            double nx = x / norm;
            double ny = y / norm;
            double nz = 1.0 / norm;
            
            // Convert 3D direction to spherical coordinates
            // theta: longitude (azimuth), phi: latitude (elevation)
            double theta = atan2(nx, nz);  // [-pi, pi]
            double phi = asin(ny);          // [-pi/2, pi/2]
            
            // Convert spherical coordinates to equirectangular pixel coordinates
            int px, py;
            sphericalToEquirect(theta, phi, equirect_width, equirect_height, px, py);
            
            // Copy pixel from equirectangular image
            if (px >= 0 && px < equirect_width && py >= 0 && py < equirect_height) {
                perspective.at<cv::Vec3b>(j, i) = equirect.at<cv::Vec3b>(py, px);
            } else {
                // Black pixel if out of bounds
                perspective.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    
    return perspective;
}

void PerspectiveConverter::publishPerspectiveImage(const cv::Mat& perspective, 
                                                   const builtin_interfaces::msg::Time& timestamp) {
    sensor_msgs::msg::Image output_msg;
    output_msg.header.stamp = timestamp;
    output_msg.header.frame_id = camera_frame_;
    output_msg.width = perspective.cols;
    output_msg.height = perspective.rows;
    output_msg.encoding = "rgb8";
    output_msg.is_bigendian = false;
    output_msg.step = perspective.cols * 3;
    
    size_t data_size = perspective.total() * perspective.elemSize();
    output_msg.data.resize(data_size);
    memcpy(output_msg.data.data(), perspective.data, data_size);
    
    perspective_pub_->publish(output_msg);
    
    RCLCPP_DEBUG(get_logger(), "Published perspective image: %dx%d", 
                perspective.cols, perspective.rows);
}

void PerspectiveConverter::calculateCameraIntrinsics() {
    // Calculate focal length from FOV and image size
    // FOV = 2 * atan(image_size / (2 * focal_length))
    // focal_length = image_size / (2 * tan(FOV / 2))
    double fov_rad = fov_degrees_ * M_PI / 180.0;
    double half_fov_tan = tan(fov_rad / 2.0);
    
    // For perspective projection, use average of width and height for focal length
    // Or use diagonal FOV - but simpler: use width for horizontal FOV
    // fx = width / (2 * tan(FOV/2))
    fx_ = static_cast<double>(output_width_) / (2.0 * half_fov_tan);
    // For square pixels, fy = fx
    fy_ = fx_;
    
    // Principal point at image center
    cx_ = static_cast<double>(output_width_) / 2.0;
    cy_ = static_cast<double>(output_height_) / 2.0;
}

void PerspectiveConverter::publishCameraInfo(const builtin_interfaces::msg::Time& timestamp) {
    sensor_msgs::msg::CameraInfo camera_info_msg;
    camera_info_msg.header.stamp = timestamp;
    camera_info_msg.header.frame_id = camera_frame_;
    
    // Image dimensions
    camera_info_msg.width = output_width_;
    camera_info_msg.height = output_height_;
    
    // Distortion model
    camera_info_msg.distortion_model = distortion_model_;
    
    // Camera matrix K (3x3 row-major)
    // [fx  0  cx]
    // [0  fy  cy]
    // [0   0   1]
    camera_info_msg.k[0] = fx_;  // fx
    camera_info_msg.k[1] = 0.0;
    camera_info_msg.k[2] = cx_;  // cx
    camera_info_msg.k[3] = 0.0;
    camera_info_msg.k[4] = fy_;  // fy
    camera_info_msg.k[5] = cy_;  // cy
    camera_info_msg.k[6] = 0.0;
    camera_info_msg.k[7] = 0.0;
    camera_info_msg.k[8] = 1.0;
    
    // Rectification matrix R (identity for unrectified images)
    camera_info_msg.r[0] = 1.0;
    camera_info_msg.r[1] = 0.0;
    camera_info_msg.r[2] = 0.0;
    camera_info_msg.r[3] = 0.0;
    camera_info_msg.r[4] = 1.0;
    camera_info_msg.r[5] = 0.0;
    camera_info_msg.r[6] = 0.0;
    camera_info_msg.r[7] = 0.0;
    camera_info_msg.r[8] = 1.0;
    
    // Projection matrix P (3x4 row-major)
    // Same as K for monocular camera
    camera_info_msg.p[0] = fx_;  // fx
    camera_info_msg.p[1] = 0.0;
    camera_info_msg.p[2] = cx_;  // cx
    camera_info_msg.p[3] = 0.0;  // Tx
    camera_info_msg.p[4] = 0.0;
    camera_info_msg.p[5] = fy_;  // fy
    camera_info_msg.p[6] = cy_;  // cy
    camera_info_msg.p[7] = 0.0;  // Ty
    camera_info_msg.p[8] = 0.0;
    camera_info_msg.p[9] = 0.0;
    camera_info_msg.p[10] = 1.0;
    camera_info_msg.p[11] = 0.0; // Tz
    
    // Distortion coefficients (plumb_bob: k1, k2, p1, p2, k3)
    // For perspective projection without distortion, all zeros
    camera_info_msg.d.resize(5);
    camera_info_msg.d[0] = 0.0;  // k1
    camera_info_msg.d[1] = 0.0;  // k2
    camera_info_msg.d[2] = 0.0;  // p1
    camera_info_msg.d[3] = 0.0;  // p2
    camera_info_msg.d[4] = 0.0;  // k3
    
    camera_info_pub_->publish(camera_info_msg);
    
    RCLCPP_DEBUG(get_logger(), "Published camera_info: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                fx_, fy_, cx_, cy_);
}

} // namespace theta_driver

