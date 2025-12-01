#include "theta_driver/polynomial_converter_lib.hpp"
#include <cmath>

namespace theta_driver {

PolynomialConverter::PolynomialConverter(const rclcpp::NodeOptions & options)
    : Node("polynomial_converter", options) {
    RCLCPP_INFO(get_logger(), "Initializing Polynomial Converter");
    onInit();
}

PolynomialConverter::~PolynomialConverter() {
    RCLCPP_INFO(get_logger(), "Shutting down Polynomial Converter");
}

void PolynomialConverter::onInit() {
    // Declare and get parameters
    declare_parameter<std::string>("camera_frame", "camera_link");
    get_parameter("camera_frame", camera_frame_);
    
    declare_parameter<std::string>("input_topic", "image_raw");
    get_parameter("input_topic", input_topic_);
    
    declare_parameter<std::string>("output_topic", "image_polynomial");
    get_parameter("output_topic", output_topic_);
    
    declare_parameter<int>("output_width", 640);
    get_parameter("output_width", output_width_);
    
    declare_parameter<int>("output_height", 480);
    get_parameter("output_height", output_height_);
    
    declare_parameter<double>("fov_degrees", 180.0);
    get_parameter("fov_degrees", fov_degrees_);
    
    declare_parameter<std::string>("camera_info_topic", "camera_info_polynomial");
    get_parameter("camera_info_topic", camera_info_topic_);
    
    // Camera intrinsics (can be set directly or calculated from FOV)
    declare_parameter<double>("fx", 0.0);  // 0 means calculate from FOV
    get_parameter("fx", fx_);
    
    declare_parameter<double>("fy", 0.0);
    get_parameter("fy", fy_);
    
    declare_parameter<double>("cx", 0.0);  // 0 means use center
    get_parameter("cx", cx_);
    
    declare_parameter<double>("cy", 0.0);
    get_parameter("cy", cy_);
    
    declare_parameter<double>("skew", 0.0);
    get_parameter("skew", skew_);
    
    // Distortion coefficients
    declare_parameter<double>("k2", 0.0);
    get_parameter("k2", k2_);
    
    declare_parameter<double>("k3", 0.0);
    get_parameter("k3", k3_);
    
    declare_parameter<double>("k4", 0.0);
    get_parameter("k4", k4_);
    
    declare_parameter<double>("k5", 0.0);
    get_parameter("k5", k5_);
    
    declare_parameter<double>("k6", 0.0);
    get_parameter("k6", k6_);
    
    declare_parameter<double>("k7", 0.0);
    get_parameter("k7", k7_);
    
    // Rotation offset to align inner circle with outer border (degrees)
    declare_parameter<double>("rotation_offset_degrees", 0.0);
    get_parameter("rotation_offset_degrees", rotation_offset_degrees_);
    
    RCLCPP_INFO(get_logger(), "Polynomial converter parameters:");
    RCLCPP_INFO(get_logger(), "  Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Camera info topic: %s", camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Output size: %dx%d", output_width_, output_height_);
    RCLCPP_INFO(get_logger(), "  FOV: %.1f degrees", fov_degrees_);
    RCLCPP_INFO(get_logger(), "  Distortion: k2=%.4f, k3=%.4f, k4=%.4f, k5=%.4f, k6=%.4f, k7=%.4f", 
                k2_, k3_, k4_, k5_, k6_, k7_);
    
    // Calculate camera intrinsics if not provided
    calculateCameraIntrinsics();
    
    // Create parameter callback to handle realtime parameter updates
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&PolynomialConverter::parametersCallback, this, std::placeholders::_1));
    
    // Create subscriber and publisher
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_topic_, 10,
        std::bind(&PolynomialConverter::imageCallback, this, std::placeholders::_1));
    
    polynomial_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10);
    
    RCLCPP_INFO(get_logger(), "Polynomial Converter initialized");
    RCLCPP_INFO(get_logger(), "  Camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f, skew=%.4f", 
                fx_, fy_, cx_, cy_, skew_);
}

void PolynomialConverter::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
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
        
        // Convert equirectangular to polynomial
        cv::Mat polynomial = equirectangularToPolynomial(
            equirect, output_width_, output_height_, fx_, fy_, cx_, cy_, skew_);
        
        // Publish polynomial image
        publishPolynomialImage(polynomial, msg->header.stamp);
        
        // Publish camera info
        publishCameraInfo(msg->header.stamp);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in image callback: %s", e.what());
    }
}

void PolynomialConverter::sphericalToEquirect(double theta, double phi,
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

cv::Mat PolynomialConverter::equirectangularToPolynomial(const cv::Mat& equirect, 
                                                         int output_width,
                                                         int output_height,
                                                         double fx, double fy, double cx, double cy, double skew) {
    cv::Mat polynomial(output_height, output_width, CV_8UC3);
    
    int equirect_width = equirect.cols;
    int equirect_height = equirect.rows;
    
    // For polynomial projection: r = fx * (θ + k2*θ² + k3*θ³ + k4*θ⁴ + k5*θ⁵ + k6*θ⁶ + k7*θ⁷)
    // where r is distance from center, θ is angle from optical axis
    
    for (int j = 0; j < output_height; j++) {
        for (int i = 0; i < output_width; i++) {
            // Convert pixel coordinates to normalized coordinates relative to center
            // Account for skew: x = (u - cx - (v-cy)*skew/fy) / fx
            double y = (j - cy) / fy;
            double x = (i - cx - y * skew) / fx;
            
            // Calculate thetad (distorted angle) from normalized coordinates
            // Based on rpg_vikit PolynomialCamera::cam2world
            const double thetad = sqrt(x * x + y * y);
            
            // Solve for true angle θ iteratively
            // thetad = θ + k2*θ² + k3*θ³ + k4*θ⁴ + k5*θ⁵ + k6*θ⁶ + k7*θ⁷
            // So: θ = thetad / (1 + k2*θ + k3*θ² + k4*θ³ + k5*θ⁴ + k6*θ⁵ + k7*θ⁶)
            double theta_angle = thetad;
            
            if (fabs(k2_) > 1e-6 || fabs(k3_) > 1e-6 || fabs(k4_) > 1e-6 || 
                fabs(k5_) > 1e-6 || fabs(k6_) > 1e-6 || fabs(k7_) > 1e-6) {
                // Iterative solution based on rpg_vikit implementation
                for (int iter = 0; iter < 7; ++iter) {
                    double theta2 = theta_angle * theta_angle;
                    double theta3 = theta2 * theta_angle;
                    double theta4 = theta3 * theta_angle;
                    double theta5 = theta4 * theta_angle;
                    double theta6 = theta5 * theta_angle;
                    
                    double denominator = 1.0 + k2_ * theta_angle + k3_ * theta2 + 
                                        k4_ * theta3 + k5_ * theta4 + k6_ * theta5 + k7_ * theta6;
                    
                    if (fabs(denominator) < 1e-10) {
                        break;  // Avoid division by zero
                    }
                    
                    theta_angle = thetad / denominator;
                }
            }
            
            // Apply scaling: scaling = tan(θ) / thetad
            // This gives us the corrected x, y coordinates
            double scaling = 1.0;
            if (thetad > 1e-8) {
                scaling = tan(theta_angle) / thetad;
            }
            
            double x_corrected = x * scaling;
            double y_corrected = y * scaling;
            
            // Calculate azimuth angle φ from x_corrected, y_corrected
            // This preserves the correct left/right mapping
            double phi_angle = atan2(y_corrected, x_corrected);
            
            // Convert to 3D direction vector using spherical coordinates
            // theta_angle: angle from optical axis (0 to π/2)
            // phi_angle: azimuth angle around z-axis (0 to 2π)
            double nx = sin(theta_angle) * cos(phi_angle);
            double ny = sin(theta_angle) * sin(phi_angle);
            double nz = cos(theta_angle);
            
            // Convert 3D direction to equirectangular spherical coordinates
            // theta: longitude (azimuth), phi: latitude (elevation)
            double theta = atan2(nx, nz);  // [-pi, pi]
            double phi = asin(ny);          // [-pi/2, pi/2]
            
            // Apply rotation offset to align inner circle with outer border
            // Apply rotation based on radius - more rotation at outer edge
            // This allows inner and outer parts to be aligned separately
            if (fabs(rotation_offset_degrees_) > 1e-6) {
                double rotation_offset_rad = rotation_offset_degrees_ * M_PI / 180.0;
                
                // Calculate normalized radius (0 = center, 1 = edge)
                double max_radius = sqrt(static_cast<double>(output_width * output_width + 
                                                              output_height * output_height)) / 2.0;
                double current_radius = sqrt(x * x + y * y) * fx;
                double normalized_radius = std::min(current_radius / max_radius, 1.0);
                
                // Apply rotation with weight based on radius
                // Use squared weight to make outer edge rotate more
                double rotation_weight = normalized_radius * normalized_radius;
                theta += rotation_offset_rad * rotation_weight;
                
                // Normalize theta to [-pi, pi]
                while (theta > M_PI) theta -= 2.0 * M_PI;
                while (theta < -M_PI) theta += 2.0 * M_PI;
            }
            
            // Convert spherical coordinates to equirectangular pixel coordinates
            int px, py;
            sphericalToEquirect(theta, phi, equirect_width, equirect_height, px, py);
            
            // Copy pixel from equirectangular image
            if (px >= 0 && px < equirect_width && py >= 0 && py < equirect_height) {
                polynomial.at<cv::Vec3b>(j, i) = equirect.at<cv::Vec3b>(py, px);
            } else {
                // Black pixel if out of bounds
                polynomial.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    
    return polynomial;
}

void PolynomialConverter::publishPolynomialImage(const cv::Mat& polynomial, 
                                                 const builtin_interfaces::msg::Time& timestamp) {
    sensor_msgs::msg::Image output_msg;
    output_msg.header.stamp = timestamp;
    output_msg.header.frame_id = camera_frame_;
    output_msg.width = polynomial.cols;
    output_msg.height = polynomial.rows;
    output_msg.encoding = "rgb8";
    output_msg.is_bigendian = false;
    output_msg.step = polynomial.cols * 3;
    
    size_t data_size = polynomial.total() * polynomial.elemSize();
    output_msg.data.resize(data_size);
    memcpy(output_msg.data.data(), polynomial.data, data_size);
    
    polynomial_pub_->publish(output_msg);
    
    RCLCPP_DEBUG(get_logger(), "Published polynomial image: %dx%d", 
                polynomial.cols, polynomial.rows);
}

void PolynomialConverter::calculateCameraIntrinsics() {
    // If intrinsics are not provided, calculate from FOV
    if (fx_ <= 0.0 || fy_ <= 0.0) {
        double fov_rad = fov_degrees_ * M_PI / 180.0;
        
        // For polynomial projection, similar to equidistant
        // At edge: r_max = width/2, θ_max = FOV/2
        // r = fx * θ (approximately for small angles)
        fx_ = static_cast<double>(output_width_) / fov_rad;
        fy_ = fx_;  // Assume square pixels
    }
    
    // Principal point at image center if not provided
    if (cx_ <= 0.0) {
        cx_ = static_cast<double>(output_width_) / 2.0;
    }
    if (cy_ <= 0.0) {
        cy_ = static_cast<double>(output_height_) / 2.0;
    }
}

void PolynomialConverter::publishCameraInfo(const builtin_interfaces::msg::Time& timestamp) {
    sensor_msgs::msg::CameraInfo camera_info_msg;
    camera_info_msg.header.stamp = timestamp;
    camera_info_msg.header.frame_id = camera_frame_;
    
    // Image dimensions
    camera_info_msg.width = output_width_;
    camera_info_msg.height = output_height_;
    
    // Distortion model for polynomial camera
    camera_info_msg.distortion_model = "polynomial";
    
    // Camera matrix K (3x3 row-major)
    camera_info_msg.k[0] = fx_;   // fx
    camera_info_msg.k[1] = skew_; // skew
    camera_info_msg.k[2] = cx_;   // cx
    camera_info_msg.k[3] = 0.0;
    camera_info_msg.k[4] = fy_;   // fy
    camera_info_msg.k[5] = cy_;   // cy
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
    camera_info_msg.p[0] = fx_;   // fx
    camera_info_msg.p[1] = skew_; // skew
    camera_info_msg.p[2] = cx_;   // cx
    camera_info_msg.p[3] = 0.0;   // Tx
    camera_info_msg.p[4] = 0.0;
    camera_info_msg.p[5] = fy_;   // fy
    camera_info_msg.p[6] = cy_;   // cy
    camera_info_msg.p[7] = 0.0;   // Ty
    camera_info_msg.p[8] = 0.0;
    camera_info_msg.p[9] = 0.0;
    camera_info_msg.p[10] = 1.0;
    camera_info_msg.p[11] = 0.0; // Tz
    
    // Distortion coefficients for polynomial model: k2, k3, k4, k5, k6, k7
    camera_info_msg.d.resize(6);
    camera_info_msg.d[0] = k2_;
    camera_info_msg.d[1] = k3_;
    camera_info_msg.d[2] = k4_;
    camera_info_msg.d[3] = k5_;
    camera_info_msg.d[4] = k6_;
    camera_info_msg.d[5] = k7_;
    
    camera_info_pub_->publish(camera_info_msg);
    
    RCLCPP_DEBUG(get_logger(), "Published camera_info: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f, skew=%.4f", 
                fx_, fy_, cx_, cy_, skew_);
}

rcl_interfaces::msg::SetParametersResult PolynomialConverter::parametersCallback(
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
        else if (name == "fx") {
            fx_ = param.as_double();
            RCLCPP_INFO(get_logger(), "Updated fx: %.2f", fx_);
        }
        else if (name == "fy") {
            fy_ = param.as_double();
            RCLCPP_INFO(get_logger(), "Updated fy: %.2f", fy_);
        }
        else if (name == "cx") {
            cx_ = param.as_double();
            RCLCPP_INFO(get_logger(), "Updated cx: %.2f", cx_);
        }
        else if (name == "cy") {
            cy_ = param.as_double();
            RCLCPP_INFO(get_logger(), "Updated cy: %.2f", cy_);
        }
        else if (name == "skew") {
            skew_ = param.as_double();
            RCLCPP_INFO(get_logger(), "Updated skew: %.6f", skew_);
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
        else if (name == "k5") {
            k5_ = param.as_double();
            RCLCPP_INFO(get_logger(), "Updated k5: %.6f", k5_);
        }
        else if (name == "k6") {
            k6_ = param.as_double();
            RCLCPP_INFO(get_logger(), "Updated k6: %.6f", k6_);
        }
        else if (name == "k7") {
            k7_ = param.as_double();
            RCLCPP_INFO(get_logger(), "Updated k7: %.6f", k7_);
        }
        else if (name == "rotation_offset_degrees") {
            rotation_offset_degrees_ = param.as_double();
            RCLCPP_INFO(get_logger(), "Updated rotation_offset_degrees: %.2f", rotation_offset_degrees_);
        }
    }
    
    return result;
}

} // namespace theta_driver

