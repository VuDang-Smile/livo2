#include "theta_driver/ocam_converter_lib.hpp"
#include <cmath>
#include <fstream>
#include <sstream>

namespace theta_driver {

OcamConverter::OcamConverter(const rclcpp::NodeOptions & options)
    : Node("ocam_converter", options) {
    RCLCPP_INFO(get_logger(), "Initializing Ocam Converter");
    model_initialized_ = false;
    onInit();
}

OcamConverter::~OcamConverter() {
    RCLCPP_INFO(get_logger(), "Shutting down Ocam Converter");
}

void OcamConverter::onInit() {
    // Declare and get parameters
    declare_parameter<std::string>("camera_frame", "camera_link");
    get_parameter("camera_frame", camera_frame_);
    
    declare_parameter<std::string>("input_topic", "image_raw");
    get_parameter("input_topic", input_topic_);
    
    declare_parameter<std::string>("output_topic", "image_ocam");
    get_parameter("output_topic", output_topic_);
    
    declare_parameter<int>("output_width", 640);
    get_parameter("output_width", output_width_);
    
    declare_parameter<int>("output_height", 480);
    get_parameter("output_height", output_height_);
    
    declare_parameter<std::string>("camera_info_topic", "camera_info_ocam");
    get_parameter("camera_info_topic", camera_info_topic_);
    
    // Calibration file (optional)
    declare_parameter<std::string>("calib_file", "");
    get_parameter("calib_file", calib_file_);
    
    // Manual parameters (if calib_file is empty)
    // pol_coeffs and invpol_coeffs can be passed as comma-separated string or array
    declare_parameter<std::string>("pol_coeffs", "");
    std::string pol_coeffs_str;
    get_parameter("pol_coeffs", pol_coeffs_str);
    if (!pol_coeffs_str.empty()) {
        // Parse comma-separated string
        std::istringstream iss(pol_coeffs_str);
        std::string token;
        pol_coeffs_.clear();
        while (std::getline(iss, token, ',')) {
            try {
                pol_coeffs_.push_back(std::stod(token));
            } catch (...) {
                // Skip invalid tokens
            }
        }
    }
    
    declare_parameter<std::string>("invpol_coeffs", "");
    std::string invpol_coeffs_str;
    get_parameter("invpol_coeffs", invpol_coeffs_str);
    if (!invpol_coeffs_str.empty()) {
        // Parse comma-separated string
        std::istringstream iss(invpol_coeffs_str);
        std::string token;
        invpol_coeffs_.clear();
        while (std::getline(iss, token, ',')) {
            try {
                invpol_coeffs_.push_back(std::stod(token));
            } catch (...) {
                // Skip invalid tokens
            }
        }
    }
    
    declare_parameter<double>("xc", 0.0);  // 0 means use center
    get_parameter("xc", xc_);
    
    declare_parameter<double>("yc", 0.0);
    get_parameter("yc", yc_);
    
    declare_parameter<double>("c", 1.0);
    get_parameter("c", c_);
    
    declare_parameter<double>("d", 0.0);
    get_parameter("d", d_);
    
    declare_parameter<double>("e", 0.0);
    get_parameter("e", e_);
    
    RCLCPP_INFO(get_logger(), "Ocam converter parameters:");
    RCLCPP_INFO(get_logger(), "  Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Camera info topic: %s", camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Output size: %dx%d", output_width_, output_height_);
    
    // Initialize Ocam model
    if (!calib_file_.empty()) {
        RCLCPP_INFO(get_logger(), "  Loading calibration file: %s", calib_file_.c_str());
        if (loadCalibFile(calib_file_)) {
            model_initialized_ = true;
        } else {
            RCLCPP_WARN(get_logger(), "Failed to load calibration file, using default parameters");
            initFromParameters();
        }
    } else {
        RCLCPP_INFO(get_logger(), "  Using manual parameters");
        initFromParameters();
    }
    
    if (!model_initialized_) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize Ocam model");
        return;
    }
    
    // Create parameter callback to handle realtime parameter updates
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&OcamConverter::parametersCallback, this, std::placeholders::_1));
    
    // Create subscriber and publisher
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_topic_, 10,
        std::bind(&OcamConverter::imageCallback, this, std::placeholders::_1));
    
    ocam_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10);
    
    RCLCPP_INFO(get_logger(), "Ocam Converter initialized");
    RCLCPP_INFO(get_logger(), "  Center: (%.2f, %.2f)", ocam_model_.xc, ocam_model_.yc);
    RCLCPP_INFO(get_logger(), "  Affine: c=%.4f, d=%.4f, e=%.4f", 
                ocam_model_.c, ocam_model_.d, ocam_model_.e);
    RCLCPP_INFO(get_logger(), "  Polynomial length: %d", ocam_model_.length_pol);
    RCLCPP_INFO(get_logger(), "  Inverse polynomial length: %d", ocam_model_.length_invpol);
}

bool OcamConverter::loadCalibFile(const std::string& calib_file) {
    std::ifstream f(calib_file);
    if (!f.is_open()) {
        RCLCPP_ERROR(get_logger(), "Cannot open calibration file: %s", calib_file.c_str());
        return false;
    }
    
    std::string line;
    
    // Read polynomial coefficients
    std::getline(f, line);  // Skip header
    f >> ocam_model_.length_pol;
    for (int i = 0; i < ocam_model_.length_pol && i < MAX_POL_LENGTH; i++) {
        f >> ocam_model_.pol[i];
    }
    
    // Read inverse polynomial coefficients
    std::getline(f, line);  // Skip newline
    std::getline(f, line);  // Skip header
    std::getline(f, line);  // Skip empty line
    f >> ocam_model_.length_invpol;
    for (int i = 0; i < ocam_model_.length_invpol && i < MAX_POL_LENGTH; i++) {
        f >> ocam_model_.invpol[i];
    }
    
    // Read center coordinates
    std::getline(f, line);  // Skip newline
    std::getline(f, line);  // Skip header
    std::getline(f, line);  // Skip empty line
    f >> ocam_model_.xc >> ocam_model_.yc;
    
    // Read affine coefficients
    std::getline(f, line);  // Skip newline
    std::getline(f, line);  // Skip header
    std::getline(f, line);  // Skip empty line
    f >> ocam_model_.c >> ocam_model_.d >> ocam_model_.e;
    
    f.close();
    return true;
}

void OcamConverter::initFromParameters() {
    // Initialize polynomial coefficients
    if (pol_coeffs_.empty()) {
        // Default: simple fisheye-like model
        // pol[0] = focal length, pol[1] = distortion
        ocam_model_.length_pol = 3;
        ocam_model_.pol[0] = 200.0;  // Base focal length
        ocam_model_.pol[1] = 0.0;     // Linear term
        ocam_model_.pol[2] = 0.0001;  // Quadratic term (distortion)
    } else {
        ocam_model_.length_pol = std::min(static_cast<int>(pol_coeffs_.size()), MAX_POL_LENGTH);
        for (int i = 0; i < ocam_model_.length_pol; i++) {
            ocam_model_.pol[i] = pol_coeffs_[i];
        }
    }
    
    // Initialize inverse polynomial coefficients
    if (invpol_coeffs_.empty()) {
        // Default: approximate inverse
        ocam_model_.length_invpol = 3;
        ocam_model_.invpol[0] = 0.005;   // Approximate inverse
        ocam_model_.invpol[1] = 0.0;
        ocam_model_.invpol[2] = -0.0000001;
    } else {
        ocam_model_.length_invpol = std::min(static_cast<int>(invpol_coeffs_.size()), MAX_POL_LENGTH);
        for (int i = 0; i < ocam_model_.length_invpol; i++) {
            ocam_model_.invpol[i] = invpol_coeffs_[i];
        }
    }
    
    // Center coordinates
    if (xc_ <= 0.0) {
        ocam_model_.xc = static_cast<double>(output_width_) / 2.0;
    } else {
        ocam_model_.xc = xc_;
    }
    
    if (yc_ <= 0.0) {
        ocam_model_.yc = static_cast<double>(output_height_) / 2.0;
    } else {
        ocam_model_.yc = yc_;
    }
    
    // Affine parameters
    ocam_model_.c = c_;
    ocam_model_.d = d_;
    ocam_model_.e = e_;
    
    model_initialized_ = true;
}

void OcamConverter::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
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
        
        // Convert equirectangular to Ocam
        cv::Mat ocam_image = equirectangularToOcam(equirect, output_width_, output_height_);
        
        // Publish Ocam image
        publishOcamImage(ocam_image, msg->header.stamp);
        
        // Publish camera info
        publishCameraInfo(msg->header.stamp);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in image callback: %s", e.what());
    }
}

void OcamConverter::sphericalToEquirect(double theta, double phi,
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

void OcamConverter::cam2world(double u, double v, double& nx, double& ny, double& nz) const {
    // Based on rpg_vikit OmniCamera::cam2world
    // Important: we exchange x and y since regular pinhole model is working with x along the columns and y along the rows
    // Davide's framework is doing exactly the opposite
    
    double invdet = 1.0 / (ocam_model_.c - ocam_model_.d * ocam_model_.e);
    
    double x = invdet * ((v - ocam_model_.xc) - ocam_model_.d * (u - ocam_model_.yc));
    double y = invdet * (-ocam_model_.e * (v - ocam_model_.xc) + ocam_model_.c * (u - ocam_model_.yc));
    
    double r = sqrt(x * x + y * y);
    double z = ocam_model_.pol[0];
    double r_i = 1.0;
    
    for (int i = 1; i < ocam_model_.length_pol; i++) {
        r_i *= r;
        z += r_i * ocam_model_.pol[i];
    }
    
    // Normalize
    double norm = sqrt(x * x + y * y + z * z);
    if (norm > 1e-8) {
        nx = y / norm;   // Exchange x and y
        ny = x / norm;
        nz = -z / norm;  // Negate z
    } else {
        nx = 0.0;
        ny = 0.0;
        nz = 1.0;
    }
}

cv::Mat OcamConverter::equirectangularToOcam(const cv::Mat& equirect, 
                                              int output_width,
                                              int output_height) {
    cv::Mat ocam_image(output_height, output_width, CV_8UC3);
    
    int equirect_width = equirect.cols;
    int equirect_height = equirect.rows;
    
    // For each pixel in output Ocam image
    for (int j = 0; j < output_height; j++) {
        for (int i = 0; i < output_width; i++) {
            // Convert pixel coordinates to Ocam world coordinates
            double nx, ny, nz;
            cam2world(static_cast<double>(i), static_cast<double>(j), nx, ny, nz);
            
            // Convert 3D direction to equirectangular spherical coordinates
            // theta: longitude (azimuth), phi: latitude (elevation)
            double theta = atan2(nx, nz);  // [-pi, pi]
            double phi = asin(ny);          // [-pi/2, pi/2]
            
            // Convert spherical coordinates to equirectangular pixel coordinates
            int px, py;
            sphericalToEquirect(theta, phi, equirect_width, equirect_height, px, py);
            
            // Copy pixel from equirectangular image
            if (px >= 0 && px < equirect_width && py >= 0 && py < equirect_height) {
                ocam_image.at<cv::Vec3b>(j, i) = equirect.at<cv::Vec3b>(py, px);
            } else {
                // Black pixel if out of bounds
                ocam_image.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    
    return ocam_image;
}

void OcamConverter::publishOcamImage(const cv::Mat& ocam_image, 
                                    const builtin_interfaces::msg::Time& timestamp) {
    sensor_msgs::msg::Image output_msg;
    output_msg.header.stamp = timestamp;
    output_msg.header.frame_id = camera_frame_;
    output_msg.width = ocam_image.cols;
    output_msg.height = ocam_image.rows;
    output_msg.encoding = "rgb8";
    output_msg.is_bigendian = false;
    output_msg.step = ocam_image.cols * 3;
    
    size_t data_size = ocam_image.total() * ocam_image.elemSize();
    output_msg.data.resize(data_size);
    memcpy(output_msg.data.data(), ocam_image.data, data_size);
    
    ocam_pub_->publish(output_msg);
    
    RCLCPP_DEBUG(get_logger(), "Published Ocam image: %dx%d", 
                ocam_image.cols, ocam_image.rows);
}

void OcamConverter::publishCameraInfo(const builtin_interfaces::msg::Time& timestamp) {
    sensor_msgs::msg::CameraInfo camera_info_msg;
    camera_info_msg.header.stamp = timestamp;
    camera_info_msg.header.frame_id = camera_frame_;
    
    // Image dimensions
    camera_info_msg.width = output_width_;
    camera_info_msg.height = output_height_;
    
    // Distortion model for Ocam
    camera_info_msg.distortion_model = "ocam";
    
    // Camera matrix K (3x3 row-major) - Ocam doesn't use standard K matrix
    camera_info_msg.k[0] = 1.0;
    camera_info_msg.k[1] = 0.0;
    camera_info_msg.k[2] = ocam_model_.xc;
    camera_info_msg.k[3] = 0.0;
    camera_info_msg.k[4] = 1.0;
    camera_info_msg.k[5] = ocam_model_.yc;
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
    camera_info_msg.p[0] = 1.0;
    camera_info_msg.p[1] = 0.0;
    camera_info_msg.p[2] = ocam_model_.xc;
    camera_info_msg.p[3] = 0.0;
    camera_info_msg.p[4] = 0.0;
    camera_info_msg.p[5] = 1.0;
    camera_info_msg.p[6] = ocam_model_.yc;
    camera_info_msg.p[7] = 0.0;
    camera_info_msg.p[8] = 0.0;
    camera_info_msg.p[9] = 0.0;
    camera_info_msg.p[10] = 1.0;
    camera_info_msg.p[11] = 0.0;
    
    // Distortion coefficients: store polynomial coefficients
    camera_info_msg.d.resize(ocam_model_.length_pol);
    for (int i = 0; i < ocam_model_.length_pol; i++) {
        camera_info_msg.d[i] = ocam_model_.pol[i];
    }
    
    camera_info_pub_->publish(camera_info_msg);
    
    RCLCPP_DEBUG(get_logger(), "Published camera_info: center=(%.2f, %.2f)", 
                ocam_model_.xc, ocam_model_.yc);
}

rcl_interfaces::msg::SetParametersResult OcamConverter::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    bool need_reinit = false;
    
    for (const auto &param : parameters) {
        const std::string &name = param.get_name();
        
        if (name == "output_width") {
            output_width_ = param.as_int();
            need_reinit = true;
            RCLCPP_INFO(get_logger(), "Updated output_width: %d", output_width_);
        }
        else if (name == "output_height") {
            output_height_ = param.as_int();
            need_reinit = true;
            RCLCPP_INFO(get_logger(), "Updated output_height: %d", output_height_);
        }
        else if (name == "xc") {
            xc_ = param.as_double();
            if (xc_ <= 0.0) {
                ocam_model_.xc = static_cast<double>(output_width_) / 2.0;
            } else {
                ocam_model_.xc = xc_;
            }
            RCLCPP_INFO(get_logger(), "Updated xc: %.2f", ocam_model_.xc);
        }
        else if (name == "yc") {
            yc_ = param.as_double();
            if (yc_ <= 0.0) {
                ocam_model_.yc = static_cast<double>(output_height_) / 2.0;
            } else {
                ocam_model_.yc = yc_;
            }
            RCLCPP_INFO(get_logger(), "Updated yc: %.2f", ocam_model_.yc);
        }
        else if (name == "c") {
            c_ = param.as_double();
            ocam_model_.c = c_;
            RCLCPP_INFO(get_logger(), "Updated c: %.4f", ocam_model_.c);
        }
        else if (name == "d") {
            d_ = param.as_double();
            ocam_model_.d = d_;
            RCLCPP_INFO(get_logger(), "Updated d: %.4f", ocam_model_.d);
        }
        else if (name == "e") {
            e_ = param.as_double();
            ocam_model_.e = e_;
            RCLCPP_INFO(get_logger(), "Updated e: %.4f", ocam_model_.e);
        }
        else if (name == "pol_coeffs") {
            std::string pol_coeffs_str = param.as_string();
            if (!pol_coeffs_str.empty()) {
                // Parse comma-separated string
                std::istringstream iss(pol_coeffs_str);
                std::string token;
                pol_coeffs_.clear();
                while (std::getline(iss, token, ',')) {
                    try {
                        pol_coeffs_.push_back(std::stod(token));
                    } catch (...) {
                        // Skip invalid tokens
                    }
                }
                if (!pol_coeffs_.empty()) {
                    ocam_model_.length_pol = std::min(static_cast<int>(pol_coeffs_.size()), MAX_POL_LENGTH);
                    for (int i = 0; i < ocam_model_.length_pol; i++) {
                        ocam_model_.pol[i] = pol_coeffs_[i];
                    }
                    RCLCPP_INFO(get_logger(), "Updated pol_coeffs, length: %d", ocam_model_.length_pol);
                }
            }
        }
        else if (name == "invpol_coeffs") {
            std::string invpol_coeffs_str = param.as_string();
            if (!invpol_coeffs_str.empty()) {
                // Parse comma-separated string
                std::istringstream iss(invpol_coeffs_str);
                std::string token;
                invpol_coeffs_.clear();
                while (std::getline(iss, token, ',')) {
                    try {
                        invpol_coeffs_.push_back(std::stod(token));
                    } catch (...) {
                        // Skip invalid tokens
                    }
                }
                if (!invpol_coeffs_.empty()) {
                    ocam_model_.length_invpol = std::min(static_cast<int>(invpol_coeffs_.size()), MAX_POL_LENGTH);
                    for (int i = 0; i < ocam_model_.length_invpol; i++) {
                        ocam_model_.invpol[i] = invpol_coeffs_[i];
                    }
                    RCLCPP_INFO(get_logger(), "Updated invpol_coeffs, length: %d", ocam_model_.length_invpol);
                }
            }
        }
    }
    
    if (need_reinit) {
        initFromParameters();
    }
    
    return result;
}

} // namespace theta_driver

