#include "theta_driver/equidistant_converter_lib.hpp"
#include <cmath>

namespace theta_driver {

EquidistantConverter::EquidistantConverter(const rclcpp::NodeOptions & options)
    : Node("equidistant_converter", options) {
    RCLCPP_INFO(get_logger(), "Initializing Equidistant Converter");
    onInit();
}

EquidistantConverter::~EquidistantConverter() {
    RCLCPP_INFO(get_logger(), "Shutting down Equidistant Converter");
}

void EquidistantConverter::onInit() {
    // Declare and get parameters
    declare_parameter<std::string>("camera_frame", "camera_link");
    get_parameter("camera_frame", camera_frame_);
    
    declare_parameter<std::string>("input_topic", "image_raw");
    get_parameter("input_topic", input_topic_);
    
    declare_parameter<std::string>("output_topic", "image_equidistant");
    get_parameter("output_topic", output_topic_);
    
    declare_parameter<int>("output_width", 640);
    get_parameter("output_width", output_width_);
    
    declare_parameter<int>("output_height", 480);
    get_parameter("output_height", output_height_);
    
    declare_parameter<double>("fov_degrees", 180.0);
    get_parameter("fov_degrees", fov_degrees_);
    
    declare_parameter<std::string>("camera_info_topic", "camera_info_equidistant");
    get_parameter("camera_info_topic", camera_info_topic_);
    
    // Distortion coefficients
    declare_parameter<double>("k1", 0.0);
    get_parameter("k1", k1_);
    
    declare_parameter<double>("k2", 0.0);
    get_parameter("k2", k2_);
    
    declare_parameter<double>("k3", 0.0);
    get_parameter("k3", k3_);
    
    declare_parameter<double>("k4", 0.0);
    get_parameter("k4", k4_);
    
    RCLCPP_INFO(get_logger(), "Equidistant converter parameters:");
    RCLCPP_INFO(get_logger(), "  Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Camera info topic: %s", camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Output size: %dx%d", output_width_, output_height_);
    RCLCPP_INFO(get_logger(), "  FOV: %.1f degrees", fov_degrees_);
    RCLCPP_INFO(get_logger(), "  Distortion: k1=%.4f, k2=%.4f, k3=%.4f, k4=%.4f", 
                k1_, k2_, k3_, k4_);
    
    // Calculate camera intrinsics
    calculateCameraIntrinsics();
    
    // Create parameter callback to handle realtime parameter updates
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&EquidistantConverter::parametersCallback, this, std::placeholders::_1));
    
    // Create subscriber and publisher
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_topic_, 10,
        std::bind(&EquidistantConverter::imageCallback, this, std::placeholders::_1));
    
    equidistant_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10);
    
    RCLCPP_INFO(get_logger(), "Equidistant Converter initialized");
    RCLCPP_INFO(get_logger(), "  Camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                fx_, fy_, cx_, cy_);
}

void EquidistantConverter::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
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
        
        // Convert equirectangular to equidistant
        cv::Mat equidistant = equirectangularToEquidistant(
            equirect, output_width_, output_height_, fx_, fy_, cx_, cy_);
        
        // Publish equidistant image
        publishEquidistantImage(equidistant, msg->header.stamp);
        
        // Publish camera info
        publishCameraInfo(msg->header.stamp);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in image callback: %s", e.what());
    }
}

void EquidistantConverter::sphericalToEquirect(double theta, double phi,
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

cv::Mat EquidistantConverter::equirectangularToEquidistant(const cv::Mat& equirect, 
                                                           int output_width,
                                                           int output_height,
                                                           double fx, double fy, double cx, double cy) {
    cv::Mat equidistant(output_height, output_width, CV_8UC3);
    
    int equirect_width = equirect.cols;
    int equirect_height = equirect.rows;
    
    // For equidistant (fisheye) projection: r = f * θ
    // where r is distance from center, θ is angle from optical axis
    
    for (int j = 0; j < output_height; j++) {
        for (int i = 0; i < output_width; i++) {
            // Convert pixel coordinates to normalized coordinates relative to center
            double x = i - cx;
            double y = j - cy;
            
            // Calculate distance from center (this is r_distorted in the output image)
            double r_distorted = sqrt(x * x + y * y);
            
            // Calculate distorted angle θ_distorted from optical axis
            // r_distorted = fx * θ_distorted, so θ_distorted = r_distorted / fx
            double theta_distorted = r_distorted / fx;
            
            // Apply inverse distortion to get the true angle θ
            // In equidistant model: r_distorted = fx * θ * (1 + k1*θ² + k2*θ⁴ + k3*θ⁶ + k4*θ⁸)
            // So: θ_distorted = θ * (1 + k1*θ² + k2*θ⁴ + k3*θ⁶ + k4*θ⁸)
            // We need to solve for θ from θ_distorted iteratively
            double theta_angle = theta_distorted;
            
            if (fabs(k1_) > 1e-6 || fabs(k2_) > 1e-6 || fabs(k3_) > 1e-6 || fabs(k4_) > 1e-6) {
                // Iterative solution: θ = θ_distorted / (1 + k1*θ² + k2*θ⁴ + k3*θ⁶ + k4*θ⁸)
                // Start with θ = θ_distorted and iterate
                for (int iter = 0; iter < 5; ++iter) {
                    double theta2 = theta_angle * theta_angle;
                    double theta4 = theta2 * theta2;
                    double theta6 = theta4 * theta2;
                    double theta8 = theta4 * theta4;
                    
                    double denominator = 1.0 + k1_ * theta2 + k2_ * theta4 + 
                                         k3_ * theta6 + k4_ * theta8;
                    
                    if (fabs(denominator) < 1e-10) {
                        break;  // Avoid division by zero
                    }
                    
                    theta_angle = theta_distorted / denominator;
                }
            }
            
            // Limit theta_angle to avoid invalid angles
            // For 180° FOV, max theta should be π/2
            if (theta_angle > M_PI / 2.0) {
                // Black pixel if out of FOV
                equidistant.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
                continue;
            }
            
            // Calculate azimuth angle φ from x, y
            double phi_angle = atan2(y, x);
            
            // Convert to spherical coordinates
            // theta: angle from optical axis (0 to π/2)
            // phi: azimuth angle (0 to 2π)
            // For front-facing camera, we need to map this to equirectangular coordinates
            
            // Convert to 3D direction vector
            // For equidistant projection, we have:
            // - theta_angle is the angle from optical axis (looking forward = z-axis)
            // - phi_angle is the azimuth angle around z-axis
            double nx = sin(theta_angle) * cos(phi_angle);
            double ny = sin(theta_angle) * sin(phi_angle);
            double nz = cos(theta_angle);
            
            // Convert 3D direction to equirectangular spherical coordinates
            // theta: longitude (azimuth), phi: latitude (elevation)
            double theta = atan2(nx, nz);  // [-pi, pi]
            double phi = asin(ny);          // [-pi/2, pi/2]
            
            // Convert spherical coordinates to equirectangular pixel coordinates
            int px, py;
            sphericalToEquirect(theta, phi, equirect_width, equirect_height, px, py);
            
            // Copy pixel from equirectangular image
            if (px >= 0 && px < equirect_width && py >= 0 && py < equirect_height) {
                equidistant.at<cv::Vec3b>(j, i) = equirect.at<cv::Vec3b>(py, px);
            } else {
                // Black pixel if out of bounds
                equidistant.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    
    return equidistant;
}

void EquidistantConverter::publishEquidistantImage(const cv::Mat& equidistant, 
                                                   const builtin_interfaces::msg::Time& timestamp) {
    sensor_msgs::msg::Image output_msg;
    output_msg.header.stamp = timestamp;
    output_msg.header.frame_id = camera_frame_;
    output_msg.width = equidistant.cols;
    output_msg.height = equidistant.rows;
    output_msg.encoding = "rgb8";
    output_msg.is_bigendian = false;
    output_msg.step = equidistant.cols * 3;
    
    size_t data_size = equidistant.total() * equidistant.elemSize();
    output_msg.data.resize(data_size);
    memcpy(output_msg.data.data(), equidistant.data, data_size);
    
    equidistant_pub_->publish(output_msg);
    
    RCLCPP_DEBUG(get_logger(), "Published equidistant image: %dx%d", 
                equidistant.cols, equidistant.rows);
}

void EquidistantConverter::calculateCameraIntrinsics() {
    // For equidistant (fisheye) camera with FOV
    // For 180° FOV, the maximum r should be at the edge: r_max = width/2
    // r = fx * θ, so at edge: width/2 = fx * (FOV/2)
    // fx = (width/2) / (FOV/2) = width / FOV
    double fov_rad = fov_degrees_ * M_PI / 180.0;
    
    // For equidistant projection, focal length relates to FOV
    // At edge of image: r_max = width/2, θ_max = FOV/2
    // r = fx * θ, so: width/2 = fx * (FOV/2)
    // fx = width / FOV (in radians)
    fx_ = static_cast<double>(output_width_) / fov_rad;
    fy_ = fx_;  // Assume square pixels
    
    // Principal point at image center
    cx_ = static_cast<double>(output_width_) / 2.0;
    cy_ = static_cast<double>(output_height_) / 2.0;
}

void EquidistantConverter::publishCameraInfo(const builtin_interfaces::msg::Time& timestamp) {
    sensor_msgs::msg::CameraInfo camera_info_msg;
    camera_info_msg.header.stamp = timestamp;
    camera_info_msg.header.frame_id = camera_frame_;
    
    // Image dimensions
    camera_info_msg.width = output_width_;
    camera_info_msg.height = output_height_;
    
    // Distortion model for equidistant camera
    camera_info_msg.distortion_model = "equidistant";
    
    // Camera matrix K (3x3 row-major)
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
    
    // Distortion coefficients for equidistant model: k1, k2, k3, k4
    camera_info_msg.d.resize(4);
    camera_info_msg.d[0] = k1_;
    camera_info_msg.d[1] = k2_;
    camera_info_msg.d[2] = k3_;
    camera_info_msg.d[3] = k4_;
    
    camera_info_pub_->publish(camera_info_msg);
    
    RCLCPP_DEBUG(get_logger(), "Published camera_info: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                fx_, fy_, cx_, cy_);
}

rcl_interfaces::msg::SetParametersResult EquidistantConverter::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto &param : parameters) {
        const std::string &name = param.get_name();
        
        if (name == "fov_degrees") {
            fov_degrees_ = param.as_double();
            calculateCameraIntrinsics();
            RCLCPP_INFO(get_logger(), "Updated fov_degrees: %.2f", fov_degrees_);
        }
        else if (name == "output_width") {
            output_width_ = param.as_int();
            calculateCameraIntrinsics();
            RCLCPP_INFO(get_logger(), "Updated output_width: %d", output_width_);
        }
        else if (name == "output_height") {
            output_height_ = param.as_int();
            calculateCameraIntrinsics();
            RCLCPP_INFO(get_logger(), "Updated output_height: %d", output_height_);
        }
        else if (name == "k1") {
            k1_ = param.as_double();
            RCLCPP_INFO(get_logger(), "Updated k1: %.6f", k1_);
        }
        else if (name == "k2") {
            k2_ = param.as_double();
            RCLCPP_INFO(get_logger(), "Updated k2: %.6f", k2_);
        }
        else if (name == "k3") {
            k3_ = param.as_double();
            RCLCPP_INFO(get_logger(), "Updated k3: %.6f", k3_);
        }
        else if (name == "k4") {
            k4_ = param.as_double();
            RCLCPP_INFO(get_logger(), "Updated k4: %.6f", k4_);
        }
    }
    
    return result;
}

} // namespace theta_driver

