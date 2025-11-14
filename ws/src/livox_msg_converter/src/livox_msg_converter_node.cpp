#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <memory>
#include <vector>
#include <string>
#include <cstring>

using std::placeholders::_1;

/**
 * Node để convert Livox CustomMsg sang PointCloud2
 * Subscribe: /livox/lidar (CustomMsg)
 * Publish: /livox/points hoặc /livox/points2 (PointCloud2)
 */
class LivoxMsgConverter : public rclcpp::Node
{
public:
  LivoxMsgConverter()
    : Node("livox_msg_converter")
  {
    // Declare parameters
    this->declare_parameter<std::string>("input_topic", "/livox/lidar");
    this->declare_parameter<std::string>("output_topic", "/livox/points");
    this->declare_parameter<std::string>("frame_id", "livox_frame");
    this->declare_parameter<bool>("use_points2_if_exists", true);

    // Get parameters
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    bool use_points2 = this->get_parameter("use_points2_if_exists").as_bool();

    // Nếu output_topic là /livox/points và use_points2 = true, dùng /livox/points2
    if (use_points2 && output_topic == "/livox/points") {
      output_topic = "/livox/points2";
      RCLCPP_WARN(this->get_logger(), 
        "Sử dụng topic /livox/points2 để tránh conflict với topic /livox/points có thể đã tồn tại");
    }

    RCLCPP_INFO(this->get_logger(), "Livox Message Converter Node");
    RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());

    // Create subscriber
    subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic,
      10,
      std::bind(&LivoxMsgConverter::custom_msg_callback, this, _1)
    );

    // Create publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic,
      10
    );

    RCLCPP_INFO(this->get_logger(), "Converter node đã khởi động thành công");
  }

private:

  /**
   * Callback khi nhận được CustomMsg
   */
  void custom_msg_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr custom_msg)
  {
    // Tạo PointCloud2 message
    sensor_msgs::msg::PointCloud2 pointcloud2_msg;
    
    // Set header
    pointcloud2_msg.header = custom_msg->header;
    pointcloud2_msg.header.frame_id = frame_id_;
    
    // Set point cloud fields (theo format của Livox)
    pointcloud2_msg.fields.resize(7);
    pointcloud2_msg.fields[0].name = "x";
    pointcloud2_msg.fields[0].offset = 0;
    pointcloud2_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud2_msg.fields[0].count = 1;
    
    pointcloud2_msg.fields[1].name = "y";
    pointcloud2_msg.fields[1].offset = 4;
    pointcloud2_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud2_msg.fields[1].count = 1;
    
    pointcloud2_msg.fields[2].name = "z";
    pointcloud2_msg.fields[2].offset = 8;
    pointcloud2_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud2_msg.fields[2].count = 1;
    
    pointcloud2_msg.fields[3].name = "intensity";
    pointcloud2_msg.fields[3].offset = 12;
    pointcloud2_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud2_msg.fields[3].count = 1;
    
    pointcloud2_msg.fields[4].name = "tag";
    pointcloud2_msg.fields[4].offset = 16;
    pointcloud2_msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
    pointcloud2_msg.fields[4].count = 1;
    
    pointcloud2_msg.fields[5].name = "line";
    pointcloud2_msg.fields[5].offset = 17;
    pointcloud2_msg.fields[5].datatype = sensor_msgs::msg::PointField::UINT8;
    pointcloud2_msg.fields[5].count = 1;
    
    pointcloud2_msg.fields[6].name = "timestamp";
    pointcloud2_msg.fields[6].offset = 18;
    pointcloud2_msg.fields[6].datatype = sensor_msgs::msg::PointField::FLOAT64;
    pointcloud2_msg.fields[6].count = 1;
    
    // Point step = size of one point = 26 bytes (4+4+4+4+1+1+8)
    pointcloud2_msg.point_step = 26;
    
    // Set dimensions
    pointcloud2_msg.height = 1;
    pointcloud2_msg.width = custom_msg->point_num;
    pointcloud2_msg.row_step = pointcloud2_msg.width * pointcloud2_msg.point_step;
    
    // Set flags
    pointcloud2_msg.is_bigendian = false;
    pointcloud2_msg.is_dense = true;
    
    // Convert points từ CustomMsg sang PointCloud2
    pointcloud2_msg.data.resize(custom_msg->point_num * pointcloud2_msg.point_step);
    
    // Struct để match với LivoxPointXyzrtlt format (packed để đảm bảo size = 26 bytes)
    #pragma pack(push, 1)
    struct LivoxPointXyzrtlt {
      float x;
      float y;
      float z;
      float reflectivity;
      uint8_t tag;
      uint8_t line;
      double timestamp;
    };
    #pragma pack(pop)
    
    for (size_t i = 0; i < custom_msg->point_num; ++i) {
      LivoxPointXyzrtlt point;
      point.x = custom_msg->points[i].x;
      point.y = custom_msg->points[i].y;
      point.z = custom_msg->points[i].z;
      point.reflectivity = static_cast<float>(custom_msg->points[i].reflectivity);
      point.tag = custom_msg->points[i].tag;
      point.line = custom_msg->points[i].line;
      // Convert offset_time (nanoseconds) to timestamp (seconds)
      // timebase là timestamp của điểm đầu tiên (nanoseconds)
      // offset_time là offset từ timebase (nanoseconds)
      point.timestamp = static_cast<double>(custom_msg->timebase + custom_msg->points[i].offset_time) / 1e9;
      
      // Copy point data vào buffer
      std::memcpy(&pointcloud2_msg.data[i * pointcloud2_msg.point_step], &point, sizeof(LivoxPointXyzrtlt));
    }
    
    // Publish PointCloud2
    publisher_->publish(pointcloud2_msg);
    
    // Log mỗi 100 messages để tránh spam
    static uint32_t msg_count = 0;
    if (++msg_count % 100 == 0) {
      RCLCPP_DEBUG(this->get_logger(), 
        "Đã convert %u messages, point_num: %u", 
        msg_count, custom_msg->point_num);
    }
  }

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::string frame_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxMsgConverter>());
  rclcpp::shutdown();
  return 0;
}
