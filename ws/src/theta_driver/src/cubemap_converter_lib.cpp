#include "theta_driver/cubemap_converter_lib.hpp"
#include <cmath>

namespace theta_driver {

CubemapConverter::CubemapConverter(const rclcpp::NodeOptions & options)
    : Node("cubemap_converter", options) {
    RCLCPP_INFO(get_logger(), "Initializing Cubemap Converter");
    onInit();
}

CubemapConverter::~CubemapConverter() {
    RCLCPP_INFO(get_logger(), "Shutting down Cubemap Converter");
}

void CubemapConverter::onInit() {
    // Declare and get parameters
    declare_parameter<std::string>("camera_frame", "camera_link");
    get_parameter("camera_frame", camera_frame_);
    
    declare_parameter<int>("face_width", 512);
    get_parameter("face_width", face_width_);
    
    declare_parameter<int>("face_height", 512);
    get_parameter("face_height", face_height_);
    
    declare_parameter<double>("fov_degrees", 90.0);
    get_parameter("fov_degrees", fov_degrees_);
    
    RCLCPP_INFO(get_logger(), "Cubemap parameters:");
    RCLCPP_INFO(get_logger(), "  Face size: %dx%d", face_width_, face_height_);
    RCLCPP_INFO(get_logger(), "  FOV: %.1f degrees", fov_degrees_);
    
    // Create subscriber and publisher
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 10,
        std::bind(&CubemapConverter::imageCallback, this, std::placeholders::_1));
    
    faces_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_faces", 10);
    
    RCLCPP_INFO(get_logger(), "Cubemap Converter initialized");
}

void CubemapConverter::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
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
        
        // Create combined image from equirectangular input
        std::vector<cv::Mat> faces;
        equirectangularToCubemap(equirect, faces, face_width_, face_height_, fov_degrees_);
        cv::Mat combined = combineFaces(faces);
        
        // Publish combined image
        publishCombinedImage(combined, msg->header.stamp, "combined");
        
        RCLCPP_INFO(get_logger(), "Published combined image");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in image callback: %s", e.what());
    }
}

void CubemapConverter::cubeToEquirect(double x, double y, int face,
                                     double& theta, double& phi) {
    // x, y are in range [-1, 1]
    // face: 0=front, 1=right, 2=back, 3=left, 4=top, 5=bottom
    
    double nx, ny, nz;
    
    switch (face) {
        case 0: // Front (looking at +Z)
            nx = x;
            ny = y;
            nz = 1.0;
            break;
        case 1: // Right (looking at +X)
            nx = 1.0;
            ny = y;
            nz = -x;
            break;
        case 2: // Back (looking at -Z)
            nx = -x;
            ny = y;
            nz = -1.0;
            break;
        case 3: // Left (looking at -X)
            nx = -1.0;
            ny = y;
            nz = x;
            break;
        case 4: // Top (looking at +Y)
            nx = x;
            ny = 1.0;
            nz = -y;
            break;
        case 5: // Bottom (looking at -Y)
            nx = x;
            ny = -1.0;
            nz = y;
            break;
        default:
            nx = ny = nz = 0;
            break;
    }
    
    // Normalize
    double norm = sqrt(nx * nx + ny * ny + nz * nz);
    nx /= norm;
    ny /= norm;
    nz /= norm;
    
    // Convert to spherical coordinates
    theta = atan2(nx, nz);  // longitude [-pi, pi]
    phi = asin(ny);         // latitude [-pi/2, pi/2]
}

cv::Mat CubemapConverter::extractCubeFace(const cv::Mat& equirect, 
                                         int face_idx,
                                         int face_width,
                                         int face_height,
                                         double fov_degrees) {
    cv::Mat face(face_height, face_width, CV_8UC3);
    
    double fov_rad = fov_degrees * M_PI / 180.0;
    double half_fov = tan(fov_rad / 2.0);
    
    int equirect_width = equirect.cols;
    int equirect_height = equirect.rows;
    
    for (int j = 0; j < face_height; j++) {
        for (int i = 0; i < face_width; i++) {
            // Convert pixel coordinates to normalized coordinates [-1, 1]
            // with adjustable FOV
            double x = (2.0 * i / face_width - 1.0) * half_fov;
            double y = (2.0 * j / face_height - 1.0) * half_fov;
            
            // Convert cube coordinates to spherical coordinates
            double theta, phi;
            cubeToEquirect(x, y, face_idx, theta, phi);
            
            // Convert spherical to equirectangular pixel coordinates
            double u = (theta / M_PI + 1.0) / 2.0;  // [0, 1]
            double v = (phi / M_PI + 0.5);           // [0, 1]
            
            int px = static_cast<int>(u * equirect_width) % equirect_width;
            int py = std::min(std::max(static_cast<int>(v * equirect_height), 0), 
                            equirect_height - 1);
            
            // Copy pixel
            if (px >= 0 && px < equirect_width && py >= 0 && py < equirect_height) {
                face.at<cv::Vec3b>(j, i) = equirect.at<cv::Vec3b>(py, px);
            }
        }
    }
    
    return face;
}

void CubemapConverter::equirectangularToCubemap(const cv::Mat& equirect, 
                                               std::vector<cv::Mat>& faces,
                                               int face_width, 
                                               int face_height,
                                               double fov_degrees) {
    faces.clear();
    faces.reserve(6);
    
    // Extract 6 faces: front, right, back, left, top, bottom
    for (int i = 0; i < 6; i++) {
        cv::Mat face = extractCubeFace(equirect, i, face_width, face_height, fov_degrees);
        faces.push_back(face);
        
        RCLCPP_DEBUG(get_logger(), "Extracted face %d (%s): %dx%d", 
                    i, face_names_[i].c_str(), face.cols, face.rows);
    }
}

cv::Mat CubemapConverter::combineFaces(const std::vector<cv::Mat>& faces) {
    if (faces.size() != 6) {
        RCLCPP_ERROR(get_logger(), "Expected 6 faces, got %zu", faces.size());
        return cv::Mat();
    }
    
    // Layout:
    //         [top]
    // [left] [front] [right] [back]
    //       [bottom]
    
    int face_width = faces[0].cols;
    int face_height = faces[0].rows;
    
    // Create combined image (4 faces wide x 3 faces tall)
    cv::Mat combined = cv::Mat::zeros(face_height * 3, face_width * 4, CV_8UC3);
    
    // Place faces
    // Row 0: top (centered at column 1)
    faces[4].copyTo(combined(cv::Rect(face_width * 1, 0, face_width, face_height)));
    
    // Row 1: left, front, right, back
    faces[3].copyTo(combined(cv::Rect(face_width * 0, face_height * 1, face_width, face_height)));
    faces[0].copyTo(combined(cv::Rect(face_width * 1, face_height * 1, face_width, face_height)));
    faces[1].copyTo(combined(cv::Rect(face_width * 2, face_height * 1, face_width, face_height)));
    faces[2].copyTo(combined(cv::Rect(face_width * 3, face_height * 1, face_width, face_height)));
    
    // Row 2: bottom (centered at column 1)
    faces[5].copyTo(combined(cv::Rect(face_width * 1, face_height * 2, face_width, face_height)));
    
    return combined;
}


void CubemapConverter::publishCombinedImage(const cv::Mat& combined, 
                                           const builtin_interfaces::msg::Time& timestamp,
                                           const std::string& description) {
    sensor_msgs::msg::Image output_msg;
    output_msg.header.stamp = timestamp;
    output_msg.header.frame_id = camera_frame_;
    output_msg.width = combined.cols;
    output_msg.height = combined.rows;
    output_msg.encoding = "rgb8";
    output_msg.is_bigendian = false;
    output_msg.step = combined.cols * 3;
    
    size_t data_size = combined.total() * combined.elemSize();
    output_msg.data.resize(data_size);
    memcpy(output_msg.data.data(), combined.data, data_size);
    
    faces_pub_->publish(output_msg);
    
    RCLCPP_DEBUG(get_logger(), "Published %s combined image: %dx%d", 
                description.c_str(), combined.cols, combined.rows);
}

} // namespace theta_driver

