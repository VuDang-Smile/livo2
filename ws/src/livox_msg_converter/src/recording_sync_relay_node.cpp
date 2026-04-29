#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>

using std::placeholders::_1;

class RecordingSyncRelay : public rclcpp::Node
{
public:
  RecordingSyncRelay()
  : Node("recording_sync_relay")
  {
    declare_parameter<std::string>("lidar_input_topic", "/livox/lidar");
    declare_parameter<std::string>("lidar_output_topic", "/record_sync/livox/lidar");
    declare_parameter<std::string>("imu_input_topic", "/livox/imu");
    declare_parameter<std::string>("imu_output_topic", "/record_sync/livox/imu");
    declare_parameter<std::string>("image_input_topic", "/image_raw");
    declare_parameter<std::string>("image_output_topic", "/record_sync/image_raw");
    declare_parameter<std::string>("camera_info_input_topic", "/camera_info");
    declare_parameter<std::string>("camera_info_output_topic", "/record_sync/camera_info");
    declare_parameter<int>("subscription_queue_size", 20);
    declare_parameter<int>("publisher_queue_size", 50);

    const auto lidar_input_topic = get_parameter("lidar_input_topic").as_string();
    const auto lidar_output_topic = get_parameter("lidar_output_topic").as_string();
    const auto imu_input_topic = get_parameter("imu_input_topic").as_string();
    const auto imu_output_topic = get_parameter("imu_output_topic").as_string();
    const auto image_input_topic = get_parameter("image_input_topic").as_string();
    const auto image_output_topic = get_parameter("image_output_topic").as_string();
    const auto camera_info_input_topic = get_parameter("camera_info_input_topic").as_string();
    const auto camera_info_output_topic = get_parameter("camera_info_output_topic").as_string();
    const auto subscription_queue_size = static_cast<std::size_t>(
      std::max<std::int64_t>(1, get_parameter("subscription_queue_size").as_int()));
    const auto publisher_queue_size = static_cast<std::size_t>(
      std::max<std::int64_t>(1, get_parameter("publisher_queue_size").as_int()));

    auto subscription_qos = rclcpp::SensorDataQoS();
    subscription_qos.keep_last(subscription_queue_size);

    auto publisher_qos = rclcpp::QoS(rclcpp::KeepLast(publisher_queue_size));
    publisher_qos.reliable();
    publisher_qos.durability_volatile();

    lidar_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(lidar_output_topic, publisher_qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_output_topic, publisher_qos);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_output_topic, publisher_qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_output_topic, publisher_qos);

    lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lidar_input_topic,
      subscription_qos,
      std::bind(&RecordingSyncRelay::lidar_callback, this, _1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_input_topic,
      subscription_qos,
      std::bind(&RecordingSyncRelay::imu_callback, this, _1));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_input_topic,
      subscription_qos,
      std::bind(&RecordingSyncRelay::image_callback, this, _1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_input_topic,
      subscription_qos,
      std::bind(&RecordingSyncRelay::camera_info_callback, this, _1));

    RCLCPP_INFO(get_logger(), "Recording sync relay started");
    RCLCPP_INFO(get_logger(), "  LiDAR: %s -> %s", lidar_input_topic.c_str(), lidar_output_topic.c_str());
    RCLCPP_INFO(get_logger(), "  IMU: %s -> %s", imu_input_topic.c_str(), imu_output_topic.c_str());
    RCLCPP_INFO(get_logger(), "  Image: %s -> %s", image_input_topic.c_str(), image_output_topic.c_str());
    RCLCPP_INFO(get_logger(), "  CameraInfo: %s -> %s", camera_info_input_topic.c_str(), camera_info_output_topic.c_str());
  }

private:
  void lidar_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    auto out = *msg;
    out.header.stamp = now();
    lidar_pub_->publish(out);
    maybe_log("LiDAR", ++lidar_count_, out.header.stamp);
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto out = *msg;
    out.header.stamp = now();
    imu_pub_->publish(out);
    maybe_log("IMU", ++imu_count_, out.header.stamp);
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto out = *msg;
    out.header.stamp = now();
    image_pub_->publish(out);
    maybe_log("Image", ++image_count_, out.header.stamp);
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    auto out = *msg;
    out.header.stamp = now();
    camera_info_pub_->publish(out);
    maybe_log("CameraInfo", ++camera_info_count_, out.header.stamp);
  }

  void maybe_log(const char * label, std::uint64_t count, const rclcpp::Time & stamp)
  {
    if (count == 1 || count % 200 == 0) {
      RCLCPP_INFO(
        get_logger(),
        "%s synced: count=%llu stamp=%.6f",
        label,
        static_cast<unsigned long long>(count),
        stamp.seconds());
    }
  }

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  std::uint64_t lidar_count_ = 0;
  std::uint64_t imu_count_ = 0;
  std::uint64_t image_count_ = 0;
  std::uint64_t camera_info_count_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RecordingSyncRelay>());
  rclcpp::shutdown();
  return 0;
}
