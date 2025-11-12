#include "theta_driver/cubemap_converter_optimized.hpp"
#include <cmath>
#include <thread>
#include <future>
#include <chrono>

namespace theta_driver {

CubemapConverterOptimized::CubemapConverterOptimized(const rclcpp::NodeOptions & options)
    : Node("cubemap_converter_optimized", options) {
    RCLCPP_INFO(this->get_logger(), "Initializing Optimized Cubemap Converter");
    onInit();
}

CubemapConverterOptimized::~CubemapConverterOptimized() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Optimized Cubemap Converter");
}

void CubemapConverterOptimized::onInit() {
    // Declare and get parameters
    this->declare_parameter<std::string>("camera_frame", "camera_link");
    this->get_parameter("camera_frame", camera_frame_);
    
    this->declare_parameter<int>("face_width", 512);
    this->get_parameter("face_width", face_width_);
    
    this->declare_parameter<int>("face_height", 512);
    this->get_parameter("face_height", face_height_);
    
    this->declare_parameter<double>("fov_degrees", 90.0);
    this->get_parameter("fov_degrees", fov_degrees_);
    
    this->declare_parameter<bool>("use_lookup_table", true);
    this->get_parameter("use_lookup_table", use_lookup_table_);
    
    this->declare_parameter<int>("num_threads", std::thread::hardware_concurrency());
    this->get_parameter("num_threads", num_threads_);
    
    RCLCPP_INFO(this->get_logger(), "Optimized Cubemap parameters:");
    RCLCPP_INFO(this->get_logger(), "  Face size: %dx%d", face_width_, face_height_);
    RCLCPP_INFO(this->get_logger(), "  FOV: %.1f degrees", fov_degrees_);
    RCLCPP_INFO(this->get_logger(), "  Use lookup table: %s", use_lookup_table_ ? "yes" : "no");
    RCLCPP_INFO(this->get_logger(), "  Threads: %d", num_threads_);
    
    // Pre-compute lookup tables for performance
    if (use_lookup_table_) {
        precomputeLookupTables();
    }
    
    // Create subscriber and publisher
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 10,
        std::bind(&CubemapConverterOptimized::imageCallback, this, std::placeholders::_1));
    
    faces_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_faces", 10);
    
    RCLCPP_INFO(this->get_logger(), "Optimized Cubemap Converter initialized");
}

void CubemapConverterOptimized::precomputeLookupTables() {
    RCLCPP_INFO(this->get_logger(), "Pre-computing lookup tables...");
    
    lookup_tables_.resize(6);
    
    for (int face = 0; face < 6; face++) {
        lookup_tables_[face].resize(face_height_ * face_width_);
        
        double fov_rad = fov_degrees_ * M_PI / 180.0;
        double half_fov = tan(fov_rad / 2.0);
        
        for (int j = 0; j < face_height_; j++) {
            for (int i = 0; i < face_width_; i++) {
                int idx = j * face_width_ + i;
                
                // Convert pixel coordinates to normalized coordinates [-1, 1]
                double x = (2.0 * i / face_width_ - 1.0) * half_fov;
                double y = (2.0 * j / face_height_ - 1.0) * half_fov;
                
                // Convert cube coordinates to spherical coordinates
                double theta, phi;
                cubeToEquirect(x, y, face, theta, phi);
                
                // Store as fixed-point integers for faster access
                lookup_tables_[face][idx].u = static_cast<int>((theta / M_PI + 1.0) * 0.5 * 65536);
                lookup_tables_[face][idx].v = static_cast<int>((phi / M_PI + 0.5) * 65536);
            }
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Lookup tables computed for %dx%d faces", face_width_, face_height_);
}

void CubemapConverterOptimized::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // Avoid unnecessary memory copy - work directly with the data
        cv::Mat equirect;
        if (msg->encoding == "rgb8") {
            equirect = cv::Mat(msg->height, msg->width, CV_8UC3, 
                             const_cast<uint8_t*>(msg->data.data()));
        } else if (msg->encoding == "bgr8") {
            cv::Mat temp(msg->height, msg->width, CV_8UC3, 
                        const_cast<uint8_t*>(msg->data.data()));
            cv::cvtColor(temp, equirect, cv::COLOR_BGR2RGB);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unsupported encoding: %s", msg->encoding.c_str());
            return;
        }
        
        // Convert equirectangular to cubemap faces using optimized method
        std::vector<cv::Mat> faces;
        if (use_lookup_table_) {
            equirectangularToCubemapOptimized(equirect, faces);
        } else {
            equirectangularToCubemap(equirect, faces, face_width_, face_height_, fov_degrees_);
        }
        
        // Combine faces into a single image
        cv::Mat combined = combineFaces(faces);
        
        // Publish combined image
        sensor_msgs::msg::Image output_msg;
        output_msg.header.stamp = msg->header.stamp;
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
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        RCLCPP_DEBUG(this->get_logger(), "Processing time: %ld ms", duration.count());
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in image callback: %s", e.what());
    }
}

void CubemapConverterOptimized::equirectangularToCubemapOptimized(const cv::Mat& equirect, 
                                                                 std::vector<cv::Mat>& faces) {
    faces.clear();
    faces.reserve(6);
    
    int equirect_width = equirect.cols;
    int equirect_height = equirect.rows;
    
    // Create faces with proper memory allocation
    for (int i = 0; i < 6; i++) {
        faces.emplace_back(face_height_, face_width_, CV_8UC3);
    }
    
    // Process faces in parallel using thread pool
    std::vector<std::future<void>> futures;
    
    for (int face = 0; face < 6; face++) {
        futures.push_back(std::async(std::launch::async, [&equirect, &faces, face, equirect_width, equirect_height, face_width = face_width_, face_height = face_height_, this]() {
            extractCubeFaceOptimized(equirect, faces[face], face, equirect_width, equirect_height, face_width, face_height);
        }));
    }
    
    // Wait for all threads to complete
    for (auto& future : futures) {
        future.wait();
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Extracted 6 faces using optimized method");
}

void CubemapConverterOptimized::extractCubeFaceOptimized(const cv::Mat& equirect, 
                                                        cv::Mat& face,
                                                        int face_idx,
                                                        int equirect_width,
                                                        int equirect_height,
                                                        int face_width,
                                                        int face_height) {
    const auto& lookup_table = lookup_tables_[face_idx];
    
    // Use OpenCV's parallel processing
    cv::parallel_for_(cv::Range(0, face_height), [&](const cv::Range& range) {
        for (int j = range.start; j < range.end; j++) {
            for (int i = 0; i < face_width; i++) {
                int idx = j * face_width + i;
                
                // Use pre-computed lookup table
                int u_fixed = lookup_table[idx].u;
                int v_fixed = lookup_table[idx].v;
                
                // Convert back to pixel coordinates with bounds checking
                int px = (u_fixed * equirect_width) >> 16;
                int py = std::min(std::max((v_fixed * equirect_height) >> 16, 0), equirect_height - 1);
                
                // Handle wrapping for longitude
                px = px % equirect_width;
                if (px < 0) px += equirect_width;
                
                // Copy pixel with bounds checking
                if (px >= 0 && px < equirect_width && py >= 0 && py < equirect_height) {
                    face.at<cv::Vec3b>(j, i) = equirect.at<cv::Vec3b>(py, px);
                }
            }
        }
    });
}

void CubemapConverterOptimized::cubeToEquirect(double x, double y, int face,
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

cv::Mat CubemapConverterOptimized::extractCubeFace(const cv::Mat& equirect, 
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

void CubemapConverterOptimized::equirectangularToCubemap(const cv::Mat& equirect, 
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
        
        RCLCPP_DEBUG(this->get_logger(), "Extracted face %d (%s): %dx%d", 
                    i, face_names_[i].c_str(), face.cols, face.rows);
    }
}

cv::Mat CubemapConverterOptimized::combineFaces(const std::vector<cv::Mat>& faces) {
    if (faces.size() != 6) {
        RCLCPP_ERROR(this->get_logger(), "Expected 6 faces, got %zu", faces.size());
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

} // namespace theta_driver
