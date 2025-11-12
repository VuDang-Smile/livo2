#include "theta_driver/qr_detector_optimized.hpp"
#include <cmath>
#include <thread>
#include <future>
#include <chrono>

namespace theta_driver {

QRDetectorOptimized::QRDetectorOptimized(const rclcpp::NodeOptions & options)
    : Node("qr_detector_optimized", options) {
    RCLCPP_INFO(this->get_logger(), "Initializing Optimized QR Detector");
    onInit();
}

QRDetectorOptimized::~QRDetectorOptimized() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Optimized QR Detector");
}

void QRDetectorOptimized::onInit() {
    // Declare and get parameters
    this->declare_parameter<std::string>("camera_frame", "camera_link");
    this->get_parameter("camera_frame", camera_frame_);
    
    this->declare_parameter<int>("face_width", 768);  // Increased for better QR detection
    this->get_parameter("face_width", face_width_);
    
    this->declare_parameter<int>("face_height", 768);  // Increased for better QR detection
    this->get_parameter("face_height", face_height_);
    
    this->declare_parameter<bool>("enable_qr_detection", true);
    this->get_parameter("enable_qr_detection", enable_qr_detection_);
    
    this->declare_parameter<double>("min_confidence", 0.3);  // Lowered for better detection
    this->get_parameter("min_confidence", min_confidence_);
    
    this->declare_parameter<int>("detection_interval", 1);  // Process every frame for better detection
    this->get_parameter("detection_interval", detection_interval_);
    
    this->declare_parameter<bool>("enable_preprocessing", true);  // Enable for better detection
    this->get_parameter("enable_preprocessing", enable_preprocessing_);
    
    this->declare_parameter<bool>("enable_multithreading", true);
    this->get_parameter("enable_multithreading", enable_multithreading_);
    
    this->declare_parameter<int>("max_faces_per_thread", 3);  // Increased for better parallelization
    this->get_parameter("max_faces_per_thread", max_faces_per_thread_);
    
    // New parameters for improved detection
    this->declare_parameter<bool>("enable_rotation_detection", true);
    this->get_parameter("enable_rotation_detection", enable_rotation_detection_);
    
    this->declare_parameter<int>("qr_scale_factor", 2);  // Scale factor for QR detection
    this->get_parameter("qr_scale_factor", qr_scale_factor_);
    
    this->declare_parameter<bool>("prioritize_top_bottom", true);  // Prioritize top and bottom faces
    this->get_parameter("prioritize_top_bottom", prioritize_top_bottom_);
    
    frame_counter_ = 0;
    
    RCLCPP_INFO(this->get_logger(), "Optimized QR Detector parameters:");
    RCLCPP_INFO(this->get_logger(), "  Face size: %dx%d", face_width_, face_height_);
    RCLCPP_INFO(this->get_logger(), "  QR detection: %s", enable_qr_detection_ ? "enabled" : "disabled");
    RCLCPP_INFO(this->get_logger(), "  Min confidence: %.2f", min_confidence_);
    RCLCPP_INFO(this->get_logger(), "  Detection interval: %d frames", detection_interval_);
    RCLCPP_INFO(this->get_logger(), "  Preprocessing: %s", enable_preprocessing_ ? "enabled" : "disabled");
    RCLCPP_INFO(this->get_logger(), "  Multithreading: %s", enable_multithreading_ ? "enabled" : "disabled");
    
    // Create subscriber and publisher
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_faces", 10,
        std::bind(&QRDetectorOptimized::imageCallback, this, std::placeholders::_1));
    
    qr_pub_ = this->create_publisher<std_msgs::msg::String>("detect_qrcode", 10);
    
    RCLCPP_INFO(this->get_logger(), "Optimized QR Detector initialized");
}

void QRDetectorOptimized::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!enable_qr_detection_) {
        return;
    }
    
    // Skip frames for performance
    frame_counter_++;
    if (frame_counter_ % detection_interval_ != 0) {
        return;
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // Avoid unnecessary memory copy - work directly with the data
        cv::Mat combined_image;
        if (msg->encoding == "rgb8") {
            combined_image = cv::Mat(msg->height, msg->width, CV_8UC3, 
                                   const_cast<uint8_t*>(msg->data.data()));
        } else if (msg->encoding == "bgr8") {
            cv::Mat temp(msg->height, msg->width, CV_8UC3, 
                        const_cast<uint8_t*>(msg->data.data()));
            cv::cvtColor(temp, combined_image, cv::COLOR_BGR2RGB);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unsupported encoding: %s", msg->encoding.c_str());
            return;
        }
        
        // Extract individual faces from combined image
        std::vector<cv::Mat> faces = extractIndividualFaces(combined_image);
        
        // Detect QR codes in all faces using optimized method
        std::vector<QRCodeResult> all_results;
        if (enable_multithreading_) {
            all_results = detectQRCodesInFacesOptimized(faces);
        } else {
            all_results = detectQRCodesInFaces(faces);
        }
        
        // Publish results
        publishQRResults(all_results);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        RCLCPP_DEBUG(this->get_logger(), "QR detection time: %ld ms", duration.count());
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in QR detection callback: %s", e.what());
    }
}

std::vector<cv::Mat> QRDetectorOptimized::extractIndividualFaces(const cv::Mat& combined_image) {
    std::vector<cv::Mat> faces;
    faces.reserve(6);
    
    // Layout: 4 faces wide x 3 faces tall
    //         [top]
    // [left] [front] [right] [back]
    //       [bottom]
    
    // Calculate actual face dimensions from the combined image
    int actual_face_width = combined_image.cols / 4;  // 4 faces wide
    int actual_face_height = combined_image.rows / 3;  // 3 faces tall
    
    // Validate dimensions
    if (combined_image.cols % 4 != 0 || combined_image.rows % 3 != 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid combined image dimensions: %dx%d. Expected multiple of 4x3", 
                     combined_image.cols, combined_image.rows);
        return faces;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Extracting faces from %dx%d image, face size: %dx%d", 
                 combined_image.cols, combined_image.rows, actual_face_width, actual_face_height);
    
    // Extract faces in order: front, right, back, left, top, bottom
    // Front (row 1, col 1)
    faces.push_back(combined_image(cv::Rect(actual_face_width * 1, actual_face_height * 1, 
                                           actual_face_width, actual_face_height)).clone());
    
    // Right (row 1, col 2)
    faces.push_back(combined_image(cv::Rect(actual_face_width * 2, actual_face_height * 1, 
                                           actual_face_width, actual_face_height)).clone());
    
    // Back (row 1, col 3)
    faces.push_back(combined_image(cv::Rect(actual_face_width * 3, actual_face_height * 1, 
                                           actual_face_width, actual_face_height)).clone());
    
    // Left (row 1, col 0)
    faces.push_back(combined_image(cv::Rect(actual_face_width * 0, actual_face_height * 1, 
                                           actual_face_width, actual_face_height)).clone());
    
    // Top (row 0, col 1) - Enhanced extraction for better QR detection
    cv::Mat top_face = combined_image(cv::Rect(actual_face_width * 1, actual_face_height * 0, 
                                              actual_face_width, actual_face_height)).clone();
    
    // Rotate top face 180 degrees for better QR detection (camera orientation)
    if (enable_rotation_detection_) {
        cv::rotate(top_face, top_face, cv::ROTATE_180);
    }
    faces.push_back(top_face);
    
    // Bottom (row 2, col 1) - Enhanced extraction for better QR detection
    cv::Mat bottom_face = combined_image(cv::Rect(actual_face_width * 1, actual_face_height * 2, 
                                                 actual_face_width, actual_face_height)).clone();
    
    // Rotate bottom face 180 degrees for better QR detection (camera orientation)
    if (enable_rotation_detection_) {
        cv::rotate(bottom_face, bottom_face, cv::ROTATE_180);
    }
    faces.push_back(bottom_face);
    
    return faces;
}

std::vector<QRCodeResult> QRDetectorOptimized::detectQRCodesInFacesOptimized(const std::vector<cv::Mat>& faces) {
    std::vector<QRCodeResult> all_results;
    
    if (faces.size() != 6) {
        RCLCPP_WARN(this->get_logger(), "Expected 6 faces, got %zu", faces.size());
        return all_results;
    }
    
    // Use multithreading for face processing
    std::vector<std::future<std::vector<QRCodeResult>>> futures;
    
    if (prioritize_top_bottom_) {
        // Process top and bottom faces first (indices 4 and 5)
        std::vector<int> priority_order = {4, 5, 0, 1, 2, 3}; // top, bottom, front, right, back, left
        
        for (int face_idx : priority_order) {
            if (face_idx < static_cast<int>(faces.size()) && face_idx < static_cast<int>(face_names_.size())) {
                futures.push_back(std::async(std::launch::async, [this, &faces, face_idx]() {
                    return detectQRCodesInFaceOptimized(faces[face_idx], face_names_[face_idx], face_idx);
                }));
            }
        }
    } else {
        // Process faces in original order
        for (size_t i = 0; i < faces.size() && i < face_names_.size(); i++) {
            futures.push_back(std::async(std::launch::async, [this, &faces, i]() {
                return detectQRCodesInFaceOptimized(faces[i], face_names_[i], static_cast<int>(i));
            }));
        }
    }
    
    // Collect results
    for (auto& future : futures) {
        std::vector<QRCodeResult> face_results = future.get();
        all_results.insert(all_results.end(), face_results.begin(), face_results.end());
    }
    
    return all_results;
}

std::vector<QRCodeResult> QRDetectorOptimized::detectQRCodesInFaces(const std::vector<cv::Mat>& faces) {
    std::vector<QRCodeResult> all_results;
    
    for (size_t i = 0; i < faces.size() && i < face_names_.size(); i++) {
        std::vector<QRCodeResult> face_results = detectQRCodesInFaceOptimized(
            faces[i], face_names_[i], static_cast<int>(i));
        
        all_results.insert(all_results.end(), face_results.begin(), face_results.end());
    }
    
    return all_results;
}

std::vector<QRCodeResult> QRDetectorOptimized::detectQRCodesInFaceOptimized(const cv::Mat& face, 
                                                                           const std::string& face_name,
                                                                           int face_index) {
    std::vector<QRCodeResult> results;
    
    try {
        // Convert to grayscale for QR detection
        cv::Mat gray_face;
        if (face.channels() == 3) {
            cv::cvtColor(face, gray_face, cv::COLOR_RGB2GRAY);
        } else {
            gray_face = face.clone();
        }
        
        // Enhanced preprocessing for top and bottom faces
        bool is_top_bottom = (face_name == "top" || face_name == "bottom");
        
        // Resize face for better performance - use larger size for top/bottom
        cv::Mat processed_face = gray_face;
        int max_size = is_top_bottom ? 1024 : 768;  // Larger size for top/bottom faces
        
        if (gray_face.cols > max_size || gray_face.rows > max_size) {
            double scale = std::min(static_cast<double>(max_size) / gray_face.cols, 
                                  static_cast<double>(max_size) / gray_face.rows);
            cv::resize(gray_face, processed_face, cv::Size(), scale, scale, cv::INTER_AREA);
        }
        
        // Try detection methods based on configuration
        std::vector<cv::Mat> test_images;
        test_images.push_back(processed_face);  // Original
        
        if (enable_preprocessing_) {
            // Add histogram equalization
            cv::Mat hist_eq;
            cv::equalizeHist(processed_face, hist_eq);
            test_images.push_back(hist_eq);
            
            // Add contrast enhancement
            cv::Mat contrast;
            cv::convertScaleAbs(processed_face, contrast, 1.3, 15);  // Increased contrast
            test_images.push_back(contrast);
            
            // Add Gaussian blur for noise reduction
            cv::Mat blurred;
            cv::GaussianBlur(processed_face, blurred, cv::Size(3, 3), 0);
            test_images.push_back(blurred);
            
            // Add sharpening for top/bottom faces
            if (is_top_bottom) {
                cv::Mat sharpened;
                cv::Mat kernel = (cv::Mat_<float>(3,3) << 
                    0, -1, 0,
                    -1, 5, -1,
                    0, -1, 0);
                cv::filter2D(processed_face, sharpened, -1, kernel);
                test_images.push_back(sharpened);
            }
        }
        
        // Scale up image for better QR detection
        if (qr_scale_factor_ > 1) {
            cv::Mat scaled_face;
            cv::resize(processed_face, scaled_face, cv::Size(), qr_scale_factor_, qr_scale_factor_, cv::INTER_CUBIC);
            test_images.push_back(scaled_face);
        }
        
        // Detect QR codes with each method
        for (const auto& test_image : test_images) {
            std::vector<cv::Point2f> points;
            std::vector<cv::Mat> straight_qrcode;
            std::vector<std::string> decoded_infos;
            
            bool detected = qr_detector_.detectAndDecodeMulti(test_image, decoded_infos, points, straight_qrcode);
            
            if (detected && !decoded_infos.empty()) {
                for (size_t i = 0; i < decoded_infos.size(); i++) {
                    // Check if this QR code was already detected to avoid duplicates
                    bool already_detected = false;
                    for (const auto& existing_result : results) {
                        if (existing_result.content == decoded_infos[i] && 
                            existing_result.face_name == face_name) {
                            already_detected = true;
                            break;
                        }
                    }
                    
                    if (!already_detected) {
                        QRCodeResult result;
                        result.content = decoded_infos[i];
                        result.face_name = face_name;
                        result.face_index = face_index;
                        result.confidence = 1.0; // OpenCV doesn't provide confidence, assume high
                        
                        // Calculate bounding box from points
                        if (points.size() >= 4) {
                            cv::Rect bbox = cv::boundingRect(points);
                            // Scale back if image was scaled up
                            if (qr_scale_factor_ > 1) {
                                bbox.x /= qr_scale_factor_;
                                bbox.y /= qr_scale_factor_;
                                bbox.width /= qr_scale_factor_;
                                bbox.height /= qr_scale_factor_;
                            }
                            result.bounding_box = bbox;
                        } else {
                            result.bounding_box = cv::Rect(0, 0, face.cols, face.rows);
                        }
                        
                        results.push_back(result);
                        
                        RCLCPP_INFO(this->get_logger(), "QR Code detected in %s face: %s", 
                                   face_name.c_str(), decoded_infos[i].c_str());
                    }
                }
                
                // If QR codes found, don't try other preprocessing methods
                if (!results.empty()) {
                    break;
                }
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Error detecting QR codes in %s face: %s", 
                   face_name.c_str(), e.what());
    }
    
    return results;
}

void QRDetectorOptimized::publishQRResults(const std::vector<QRCodeResult>& results) {
    if (results.empty()) {
        return;
    }
    
    // Create a combined message with all detected QR codes
    std::string combined_message;
    
    for (const auto& result : results) {
        if (!combined_message.empty()) {
            combined_message += " | ";
        }
        
        combined_message += result.content;
    }
    
    // Publish the combined message
    std_msgs::msg::String qr_msg;
    qr_msg.data = combined_message;
    qr_pub_->publish(qr_msg);
    
    RCLCPP_INFO(this->get_logger(), "Published QR detection results: %s", combined_message.c_str());
}

} // namespace theta_driver
