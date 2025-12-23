#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <fstream>
#include <sys/stat.h>

class PCDViewerNode : public rclcpp::Node
{
public:
  PCDViewerNode() : Node("pcd_viewer_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("pcd_file", "");
    this->declare_parameter<std::string>("topic_name", "/pcd_map");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("publish_rate", 1.0);
    this->declare_parameter<bool>("loop", false);

    // Get parameters
    std::string pcd_file = this->get_parameter("pcd_file").as_string();
    std::string topic_name = this->get_parameter("topic_name").as_string();
    std::string frame_id = this->get_parameter("frame_id").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    bool loop = this->get_parameter("loop").as_bool();

    // Helper function to check if file exists
    auto file_exists = [](const std::string& path) -> bool {
      struct stat buffer;
      return (stat(path.c_str(), &buffer) == 0);
    };

    // If pcd_file is empty, try to find PCD files in Log/PCD directory
    if (pcd_file.empty()) {
      std::string log_dir = std::string(ROOT_DIR) + "Log/PCD/";
      std::string raw_file = log_dir + "all_raw_points.pcd";
      std::string downsampled_file = log_dir + "all_downsampled_points.pcd";
      
      if (file_exists(raw_file)) {
        pcd_file = raw_file;
        RCLCPP_INFO(this->get_logger(), "Found PCD file: %s", pcd_file.c_str());
      } else if (file_exists(downsampled_file)) {
        pcd_file = downsampled_file;
        RCLCPP_INFO(this->get_logger(), "Found PCD file: %s", pcd_file.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "No PCD file found in %s", log_dir.c_str());
        RCLCPP_ERROR(this->get_logger(), "Tried: %s and %s", raw_file.c_str(), downsampled_file.c_str());
        return;
      }
    }

    // Load PCD file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity(new pcl::PointCloud<pcl::PointXYZI>);
    bool loaded = false;

    // Try loading as PointXYZRGB first
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file, *cloud_rgb) == 0) {
      loaded = true;
      cloud_rgb_ = cloud_rgb;
      RCLCPP_INFO(this->get_logger(), "Loaded PCD file as PointXYZRGB: %s (%zu points)", 
                  pcd_file.c_str(), cloud_rgb->points.size());
    }
    // Try loading as PointXYZI if RGB failed
    else if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *cloud_intensity) == 0) {
      loaded = true;
      cloud_intensity_ = cloud_intensity;
      RCLCPP_INFO(this->get_logger(), "Loaded PCD file as PointXYZI: %s (%zu points)", 
                  pcd_file.c_str(), cloud_intensity->points.size());
    }
    // Try loading as PointXYZ if both failed
    else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud_xyz) == 0) {
        loaded = true;
        // Convert to PointXYZRGB for visualization
        cloud_rgb->points.resize(cloud_xyz->points.size());
        for (size_t i = 0; i < cloud_xyz->points.size(); ++i) {
          cloud_rgb->points[i].x = cloud_xyz->points[i].x;
          cloud_rgb->points[i].y = cloud_xyz->points[i].y;
          cloud_rgb->points[i].z = cloud_xyz->points[i].z;
          cloud_rgb->points[i].r = 255;
          cloud_rgb->points[i].g = 255;
          cloud_rgb->points[i].b = 255;
        }
        cloud_rgb->width = cloud_xyz->width;
        cloud_rgb->height = cloud_xyz->height;
        cloud_rgb->is_dense = cloud_xyz->is_dense;
        cloud_rgb_ = cloud_rgb;
        RCLCPP_INFO(this->get_logger(), "Loaded PCD file as PointXYZ and converted to RGB: %s (%zu points)", 
                    pcd_file.c_str(), cloud_xyz->points.size());
      }
    }

    if (!loaded) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", pcd_file.c_str());
      return;
    }

    // Create publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 10);
    frame_id_ = frame_id;
    loop_ = loop;

    // Create timer for publishing
    if (publish_rate > 0.0) {
      auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
      timer_ = this->create_wall_timer(period, std::bind(&PCDViewerNode::publishCloud, this));
    } else {
      // Publish once
      publishCloud();
    }

    RCLCPP_INFO(this->get_logger(), "PCD Viewer Node started. Publishing to topic: %s", topic_name.c_str());
  }

private:
  void publishCloud()
  {
    sensor_msgs::msg::PointCloud2 msg;
    
    if (cloud_rgb_ && !cloud_rgb_->points.empty()) {
      pcl::toROSMsg(*cloud_rgb_, msg);
    } else if (cloud_intensity_ && !cloud_intensity_->points.empty()) {
      // Convert PointXYZI to PointXYZRGB for better visualization
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
      cloud_rgb->points.resize(cloud_intensity_->points.size());
      for (size_t i = 0; i < cloud_intensity_->points.size(); ++i) {
        cloud_rgb->points[i].x = cloud_intensity_->points[i].x;
        cloud_rgb->points[i].y = cloud_intensity_->points[i].y;
        cloud_rgb->points[i].z = cloud_intensity_->points[i].z;
        // Use intensity to color the points
        float intensity = cloud_intensity_->points[i].intensity;
        uint8_t color = static_cast<uint8_t>(std::min(255.0f, intensity * 10.0f));
        cloud_rgb->points[i].r = color;
        cloud_rgb->points[i].g = color;
        cloud_rgb->points[i].b = 255;
      }
      cloud_rgb->width = cloud_intensity_->width;
      cloud_rgb->height = cloud_intensity_->height;
      cloud_rgb->is_dense = cloud_intensity_->is_dense;
      pcl::toROSMsg(*cloud_rgb, msg);
    } else {
      RCLCPP_WARN(this->get_logger(), "No point cloud data to publish");
      return;
    }

    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    publisher_->publish(msg);
    
    if (!loop_ && publish_count_++ > 0) {
      RCLCPP_INFO(this->get_logger(), "Published point cloud once. Set loop=true to publish continuously.");
      timer_->cancel();
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity_;
  std::string frame_id_;
  bool loop_;
  int publish_count_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCDViewerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

