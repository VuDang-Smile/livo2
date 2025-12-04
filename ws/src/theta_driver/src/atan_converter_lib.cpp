#include "theta_driver/atan_converter_lib.hpp"
#include <cmath>

namespace theta_driver {

ATANConverter::ATANConverter(const rclcpp::NodeOptions & options)
    : Node("atan_converter", options) {
    RCLCPP_INFO(get_logger(), "Initializing ATAN Converter");
    onInit();
}

ATANConverter::~ATANConverter() {
    RCLCPP_INFO(get_logger(), "Shutting down ATAN Converter");
}

void ATANConverter::onInit() {
    // Declare and get parameters
    declare_parameter<std::string>("camera_frame", "camera_link");
    get_parameter("camera_frame", camera_frame_);
    
    declare_parameter<std::string>("input_topic", "image_raw");
    get_parameter("input_topic", input_topic_);
    
    declare_parameter<std::string>("output_topic", "image_atan");
    get_parameter("output_topic", output_topic_);
    
    declare_parameter<int>("output_width", 640);
    get_parameter("output_width", output_width_);
    
    declare_parameter<int>("output_height", 480);
    get_parameter("output_height", output_height_);
    
    declare_parameter<std::string>("camera_info_topic", "camera_info_atan");
    get_parameter("camera_info_topic", camera_info_topic_);
    
    // Camera intrinsics (normalized, like rpg_vikit ATANCamera)
    declare_parameter<double>("fx", 0.0);  // 0 means calculate from FOV
    get_parameter("fx", fx_);
    
    declare_parameter<double>("fy", 0.0);
    get_parameter("fy", fy_);
    
    declare_parameter<double>("cx", 0.0);  // 0 means use center (normalized)
    get_parameter("cx", cx_);
    
    declare_parameter<double>("cy", 0.0);
    get_parameter("cy", cy_);
    
    // ATAN distortion parameter (d0 = s)
    declare_parameter<double>("d0", 0.0);
    get_parameter("d0", s_);
    
    // Initialize distortion parameters
    if (s_ != 0.0) {
        tans_ = 2.0 * tan(s_ / 2.0);
        tans_inv_ = 1.0 / tans_;
        s_inv_ = 1.0 / s_;
        distortion_ = true;
    } else {
        s_inv_ = 0.0;
        tans_ = 0.0;
        tans_inv_ = 0.0;
        distortion_ = false;
    }
    
    RCLCPP_INFO(get_logger(), "ATAN converter parameters:");
    RCLCPP_INFO(get_logger(), "  Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Output size: %dx%d", output_width_, output_height_);
    RCLCPP_INFO(get_logger(), "  Distortion d0 (s): %.6f", s_);
    
    // Calculate camera intrinsics if not provided
    calculateCameraIntrinsics();
    
    // Create parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ATANConverter::parametersCallback, this, std::placeholders::_1));
    
    // Create subscriber and publisher
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_topic_, 10,
        std::bind(&ATANConverter::imageCallback, this, std::placeholders::_1));
    
    atan_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10);
    
    RCLCPP_INFO(get_logger(), "ATAN Converter initialized");
    RCLCPP_INFO(get_logger(), "  fx=%.6f, fy=%.6f, cx=%.6f, cy=%.6f", fx_, fy_, cx_, cy_);
}

void ATANConverter::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
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
        
        // Convert equirectangular to ATAN
        cv::Mat atan_image = equirectangularToATAN(equirect, output_width_, output_height_);
        
        // Publish ATAN image
        publishATANImage(atan_image, msg->header.stamp);
        
        // Publish camera info
        publishCameraInfo(msg->header.stamp);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in image callback: %s", e.what());
    }
}

void ATANConverter::sphericalToEquirect(double theta, double phi,
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

double ATANConverter::invrtrans(double r_distorted) const {
    // Inverse radial distortion: returns un-distorted radius from distorted
    // Based on rpg_vikit: invrtrans(r) = tan(r * s) * tans_inv_
    if (s_ == 0.0) {
        return r_distorted;
    }
    return (tan(r_distorted * s_) * tans_inv_);
}

void ATANConverter::cam2world(double u, double v, double& nx, double& ny, double& nz) const {
    // Based on rpg_vikit ATANCamera::cam2world
    // rpg_vikit uses: fx_ = width * fx (normalized), cx_ = cx * width - 0.5
    // So we need to convert pixel coordinates to match this format
    
    // Convert pixel coordinates to rpg_vikit format
    // cx_pixel = cx_normalized * width - 0.5
    double cx_pixel = cx_ * static_cast<double>(output_width_) - 0.5;
    double cy_pixel = cy_ * static_cast<double>(output_height_) - 0.5;
    double fx_pixel = fx_ * static_cast<double>(output_width_);
    double fy_pixel = fy_ * static_cast<double>(output_height_);
    
    // Normalize coordinates (like rpg_vikit: (x - cx) * fx_inv)
    double dist_cam_x = (u - cx_pixel) / fx_pixel;
    double dist_cam_y = (v - cy_pixel) / fy_pixel;
    
    // Calculate distorted radius
    double dist_r = sqrt(dist_cam_x * dist_cam_x + dist_cam_y * dist_cam_y);
    
    // Apply inverse distortion to get true radius
    double r = invrtrans(dist_r);
    
    // Calculate correction factor
    double d_factor;
    if (dist_r > 0.01) {
        d_factor = r / dist_r;
    } else {
        d_factor = 1.0;
    }
    
    // Apply correction
    double x_corrected = dist_cam_x * d_factor;
    double y_corrected = dist_cam_y * d_factor;
    
    // Convert to 3D direction vector (unproject2d equivalent: (x, y, 1.0) normalized)
    double norm = sqrt(x_corrected * x_corrected + y_corrected * y_corrected + 1.0);
    if (norm > 1e-8) {
        nx = x_corrected / norm;
        ny = y_corrected / norm;
        nz = 1.0 / norm;
    } else {
        nx = 0.0;
        ny = 0.0;
        nz = 1.0;
    }
}

cv::Mat ATANConverter::equirectangularToATAN(const cv::Mat& equirect, 
                                             int output_width,
                                             int output_height) {
    cv::Mat atan_image(output_height, output_width, CV_8UC3);
    
    int equirect_width = equirect.cols;
    int equirect_height = equirect.rows;
    
    // For each pixel in output ATAN image
    for (int j = 0; j < output_height; j++) {
        for (int i = 0; i < output_width; i++) {
            // Convert pixel coordinates to 3D direction using ATAN model
            double nx, ny, nz;
            cam2world(static_cast<double>(i), static_cast<double>(j), nx, ny, nz);
            
            // Convert 3D direction to equirectangular spherical coordinates
            double theta = atan2(nx, nz);  // [-pi, pi]
            double phi = asin(ny);          // [-pi/2, pi/2]
            
            // Convert spherical coordinates to equirectangular pixel coordinates
            int px, py;
            sphericalToEquirect(theta, phi, equirect_width, equirect_height, px, py);
            
            // Copy pixel from equirectangular image
            if (px >= 0 && px < equirect_width && py >= 0 && py < equirect_height) {
                atan_image.at<cv::Vec3b>(j, i) = equirect.at<cv::Vec3b>(py, px);
            } else {
                // Black pixel if out of bounds
                atan_image.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    
    return atan_image;
}

void ATANConverter::publishATANImage(const cv::Mat& atan_image, 
                                    const builtin_interfaces::msg::Time& timestamp) {
    sensor_msgs::msg::Image output_msg;
    output_msg.header.stamp = timestamp;
    output_msg.header.frame_id = camera_frame_;
    output_msg.width = atan_image.cols;
    output_msg.height = atan_image.rows;
    output_msg.encoding = "rgb8";
    output_msg.is_bigendian = false;
    output_msg.step = atan_image.cols * 3;
    
    size_t data_size = atan_image.total() * atan_image.elemSize();
    output_msg.data.resize(data_size);
    memcpy(output_msg.data.data(), atan_image.data, data_size);
    
    atan_pub_->publish(output_msg);
    
    RCLCPP_DEBUG(get_logger(), "Published ATAN image: %dx%d", 
                atan_image.cols, atan_image.rows);
}

void ATANConverter::calculateCameraIntrinsics() {
    // rpg_vikit ATANCamera uses normalized intrinsics (0-1 range)
    // fx_ = width * fx (normalized), so fx (normalized) = fx_ / width
    // For default: calculate from reasonable FOV
    
    if (fx_ <= 0.0 || fy_ <= 0.0) {
        // Default: assume ~90 degree FOV
        // For normalized: fx = 1.0 / (2 * tan(FOV/2))
        // With FOV = 90°: fx = 1.0 / (2 * tan(45°)) = 1.0 / 2.0 = 0.5
        fx_ = 0.5;  // Normalized focal length
        fy_ = 0.5;
    } else {
        // User provided values are already normalized
        // But if they're > 1, assume they're in pixels and normalize
        if (fx_ > 1.0) {
            fx_ = fx_ / static_cast<double>(output_width_);
        }
        if (fy_ > 1.0) {
            fy_ = fy_ / static_cast<double>(output_height_);
        }
    }
    
    fx_inv_ = 1.0 / fx_;
    fy_inv_ = 1.0 / fy_;
    
    // Principal point (normalized 0-1)
    if (cx_ <= 0.0) {
        cx_ = 0.5;  // Center
    } else if (cx_ > 1.0) {
        cx_ = cx_ / static_cast<double>(output_width_);
    }
    
    if (cy_ <= 0.0) {
        cy_ = 0.5;  // Center
    } else if (cy_ > 1.0) {
        cy_ = cy_ / static_cast<double>(output_height_);
    }
    
    // Convert to pixel coordinates (like rpg_vikit: cx_ = cx * width - 0.5)
    // But we'll keep normalized for calculations
}

void ATANConverter::publishCameraInfo(const builtin_interfaces::msg::Time& timestamp) {
    sensor_msgs::msg::CameraInfo camera_info_msg;
    camera_info_msg.header.stamp = timestamp;
    camera_info_msg.header.frame_id = camera_frame_;
    
    // Image dimensions
    camera_info_msg.width = output_width_;
    camera_info_msg.height = output_height_;
    
    // Distortion model
    camera_info_msg.distortion_model = "atan";
    
    // Camera matrix K (convert normalized to pixel coordinates)
    // fx_pixel = fx_normalized * width, cx_pixel = cx_normalized * width - 0.5
    double fx_pixel = fx_ * static_cast<double>(output_width_);
    double fy_pixel = fy_ * static_cast<double>(output_height_);
    double cx_pixel = cx_ * static_cast<double>(output_width_) - 0.5;
    double cy_pixel = cy_ * static_cast<double>(output_height_) - 0.5;
    
    camera_info_msg.k[0] = fx_pixel;
    camera_info_msg.k[1] = 0.0;
    camera_info_msg.k[2] = cx_pixel;
    camera_info_msg.k[3] = 0.0;
    camera_info_msg.k[4] = fy_pixel;
    camera_info_msg.k[5] = cy_pixel;
    camera_info_msg.k[6] = 0.0;
    camera_info_msg.k[7] = 0.0;
    camera_info_msg.k[8] = 1.0;
    
    // Rectification matrix R (identity)
    camera_info_msg.r[0] = 1.0;
    camera_info_msg.r[1] = 0.0;
    camera_info_msg.r[2] = 0.0;
    camera_info_msg.r[3] = 0.0;
    camera_info_msg.r[4] = 1.0;
    camera_info_msg.r[5] = 0.0;
    camera_info_msg.r[6] = 0.0;
    camera_info_msg.r[7] = 0.0;
    camera_info_msg.r[8] = 1.0;
    
    // Projection matrix P
    camera_info_msg.p[0] = fx_pixel;
    camera_info_msg.p[1] = 0.0;
    camera_info_msg.p[2] = cx_pixel;
    camera_info_msg.p[3] = 0.0;
    camera_info_msg.p[4] = 0.0;
    camera_info_msg.p[5] = fy_pixel;
    camera_info_msg.p[6] = cy_pixel;
    camera_info_msg.p[7] = 0.0;
    camera_info_msg.p[8] = 0.0;
    camera_info_msg.p[9] = 0.0;
    camera_info_msg.p[10] = 1.0;
    camera_info_msg.p[11] = 0.0;
    
    // Distortion coefficients: d0 (s)
    camera_info_msg.d.resize(1);
    camera_info_msg.d[0] = s_;
    
    camera_info_pub_->publish(camera_info_msg);
    
    RCLCPP_DEBUG(get_logger(), "Published camera_info: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f, d0=%.6f", 
                fx_pixel, fy_pixel, cx_pixel, cy_pixel, s_);
}

rcl_interfaces::msg::SetParametersResult ATANConverter::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto &param : parameters) {
        const std::string &name = param.get_name();
        
        if (name == "output_width") {
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
            if (fx_ > 1.0) {
                fx_ = fx_ / static_cast<double>(output_width_);
            }
            fx_inv_ = 1.0 / fx_;
            RCLCPP_INFO(get_logger(), "Updated fx: %.6f (normalized)", fx_);
        }
        else if (name == "fy") {
            fy_ = param.as_double();
            if (fy_ > 1.0) {
                fy_ = fy_ / static_cast<double>(output_height_);
            }
            fy_inv_ = 1.0 / fy_;
            RCLCPP_INFO(get_logger(), "Updated fy: %.6f (normalized)", fy_);
        }
        else if (name == "cx") {
            cx_ = param.as_double();
            if (cx_ > 1.0) {
                cx_ = cx_ / static_cast<double>(output_width_);
            }
            RCLCPP_INFO(get_logger(), "Updated cx: %.6f (normalized)", cx_);
        }
        else if (name == "cy") {
            cy_ = param.as_double();
            if (cy_ > 1.0) {
                cy_ = cy_ / static_cast<double>(output_height_);
            }
            RCLCPP_INFO(get_logger(), "Updated cy: %.6f (normalized)", cy_);
        }
        else if (name == "d0") {
            s_ = param.as_double();
            if (s_ != 0.0) {
                tans_ = 2.0 * tan(s_ / 2.0);
                tans_inv_ = 1.0 / tans_;
                s_inv_ = 1.0 / s_;
                distortion_ = true;
            } else {
                s_inv_ = 0.0;
                tans_ = 0.0;
                tans_inv_ = 0.0;
                distortion_ = false;
            }
            RCLCPP_INFO(get_logger(), "Updated d0: %.6f", s_);
        }
    }
    
    return result;
}

} // namespace theta_driver

