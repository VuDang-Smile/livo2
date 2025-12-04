#ifndef OCAM_CONVERTER_LIB_HPP
#define OCAM_CONVERTER_LIB_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

namespace theta_driver {

#define MAX_POL_LENGTH 64

struct OcamModel {
    double pol[MAX_POL_LENGTH];      // polynomial coefficients
    int length_pol;                  // length of polynomial
    double invpol[MAX_POL_LENGTH];   // inverse polynomial coefficients
    int length_invpol;               // length of inverse polynomial
    double xc;                       // row coordinate of center
    double yc;                       // column coordinate of center
    double c;                        // affine parameter
    double d;                        // affine parameter
    double e;                        // affine parameter
};

class OcamConverter : public rclcpp::Node {
public:
    OcamConverter(const rclcpp::NodeOptions & options);
    virtual ~OcamConverter();
    
private:
    void onInit();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    // Load Ocam model from calibration file
    bool loadCalibFile(const std::string& calib_file);
    
    // Initialize Ocam model from parameters
    void initFromParameters();
    
    // Convert equirectangular to Ocam
    cv::Mat equirectangularToOcam(const cv::Mat& equirect, 
                                   int output_width,
                                   int output_height);
    
    // Convert spherical coordinates to equirectangular pixel coordinates
    void sphericalToEquirect(double theta, double phi,
                             int equirect_width, int equirect_height,
                             int& px, int& py);
    
    // Ocam cam2world function (based on rpg_vikit)
    void cam2world(double u, double v, double& nx, double& ny, double& nz) const;
    
    // Publish Ocam image
    void publishOcamImage(const cv::Mat& ocam_image, 
                         const builtin_interfaces::msg::Time& timestamp);
    
    // Publish camera info
    void publishCameraInfo(const builtin_interfaces::msg::Time& timestamp);
    
    // Parameter callback for realtime parameter updates
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ocam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    std::string camera_frame_;
    std::string input_topic_;
    std::string output_topic_;
    std::string camera_info_topic_;
    int output_width_;
    int output_height_;
    
    // Ocam model
    OcamModel ocam_model_;
    bool model_initialized_;
    
    // Calibration file path (optional)
    std::string calib_file_;
    
    // Parameters for manual configuration
    std::vector<double> pol_coeffs_;      // polynomial coefficients
    std::vector<double> invpol_coeffs_;    // inverse polynomial coefficients
    double xc_, yc_;                       // center coordinates
    double c_, d_, e_;                     // affine parameters
};

} // namespace theta_driver

#endif // OCAM_CONVERTER_LIB_HPP

