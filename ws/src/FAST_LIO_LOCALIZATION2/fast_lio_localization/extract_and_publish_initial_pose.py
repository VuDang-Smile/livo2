#!/usr/bin/env python3
"""
Script để extract initial pose từ bag file và publish lên /initialpose topic
Giúp hệ thống localization bắt đầu ngay mà không cần global localization khi replay
"""

import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import sys
import time

# Try to import rosbag2_py (optional)
try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    HAS_ROSBAG2 = True
except ImportError:
    HAS_ROSBAG2 = False
    print("⚠️  rosbag2_py not available. Will use default pose (0, 0, 0).")


class ExtractAndPublishInitialPose(Node):
    def __init__(self):
        super().__init__("extract_and_publish_initial_pose")
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.initial_pose_published = False

    def extract_first_pose_from_bag(self, bag_path, topic_name="/Odometry"):
        """Extract first pose from bag file
        
        LƯU Ý: Pose trong /Odometry là pose trong frame camera_init (odometry frame), 
        KHÔNG PHẢI pose trong map frame. Khi publish lên /initialpose, hệ thống cần 
        pose trong map frame để biết vị trí trong map.
        
        Vì vậy, KHÔNG extract từ /Odometry khi replay. Hệ thống sẽ tự tìm vị trí 
        bằng global localization (ScanContext + ICP).
        """
        # DISABLED: Không extract pose từ /Odometry vì nó là pose trong odometry frame, 
        # không phải map frame. Hệ thống sẽ tự tìm vị trí bằng global localization.
        self.get_logger().warn("⚠️ Không extract pose từ bag file vì pose trong /Odometry là odometry frame, không phải map frame")
        self.get_logger().info("   Hệ thống sẽ tự tìm vị trí bằng global localization (ScanContext + ICP)")
        return None
        
        # Code cũ (đã disable):
        # if not HAS_ROSBAG2:
        #     self.get_logger().warn("rosbag2_py not available, cannot extract pose from bag")
        #     return None
        #     
        # try:
        #     reader = rosbag2_py.SequentialReader()
        #     storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        #     converter_options = rosbag2_py.ConverterOptions(
        #         input_serialization_format='cdr',
        #         output_serialization_format='cdr'
        #     )
        #     reader.open(storage_options, converter_options)
        #     
        #     # Get topic types
        #     topic_types = reader.get_all_topics_and_types()
        #     topic_type_map = {topic.name: topic.type for topic in topic_types}
        #     
        #     # Find the topic
        #     if topic_name not in topic_type_map:
        #         self.get_logger().warn(f"Topic {topic_name} not found in bag. Available topics: {list(topic_type_map.keys())}")
        #         # Try alternative topics
        #         for alt_topic in ["/odom", "/Odometry", "/localization", "/pose"]:
        #             if alt_topic in topic_type_map:
        #                 topic_name = alt_topic
        #                 self.get_logger().info(f"Using alternative topic: {topic_name}")
        #                 break
        #         else:
        #             return None
        #     
        #     msg_type = get_message(topic_type_map[topic_name])
        #     
        #     # Read first message
        #     while reader.has_next():
        #         (topic, data, timestamp) = reader.read_next()
        #         if topic == topic_name:
        #             msg = deserialize_message(data, msg_type)
        #             if isinstance(msg, Odometry):
        #                 return msg.pose.pose
        #             else:
        #                 # Try to extract pose from message
        #                 if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
        #                     return msg.pose.pose
        #                 elif hasattr(msg, 'position') and hasattr(msg, 'orientation'):
        #                     # Create a Pose from position and orientation
        #                     pose = Pose()
        #                     pose.position = msg.position
        #                     pose.orientation = msg.orientation
        #                     return pose
        #     
        #     return None
        #     
        # except Exception as e:
        #     self.get_logger().error(f"Error extracting pose from bag: {e}")
        #     return None

    def publish_initial_pose(self, pose, frame_id="camera_init"):
        """Publish initial pose"""
        if pose is None:
            self.get_logger().error("Cannot publish None pose")
            return False
        
        try:
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            msg.pose.pose = pose
            
            # Set covariance (small covariance for high confidence)
            msg.pose.covariance[0] = 0.25  # x
            msg.pose.covariance[7] = 0.25  # y
            msg.pose.covariance[14] = 0.25  # z
            msg.pose.covariance[21] = 0.06853891945200942  # roll
            msg.pose.covariance[28] = 0.06853891945200942  # pitch
            msg.pose.covariance[35] = 0.06853891945200942  # yaw
            
            # Wait for publisher to be ready
            time.sleep(0.5)
            
            # Publish multiple times to ensure it's received
            for _ in range(3):
                self.pub_pose.publish(msg)
                time.sleep(0.1)
            
            self.get_logger().info(f"Published initial pose: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
            self.get_logger().info(f"  Orientation: x={pose.orientation.x:.3f}, y={pose.orientation.y:.3f}, z={pose.orientation.z:.3f}, w={pose.orientation.w:.3f}")
            self.initial_pose_published = True
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error publishing initial pose: {e}")
            return False


def main(args=None):
    parser = argparse.ArgumentParser(description="Extract and publish initial pose from bag file")
    parser.add_argument("bag_path", type=str, help="Path to bag file/folder")
    parser.add_argument("--topic", type=str, default="/Odometry", help="Topic name to extract pose from")
    parser.add_argument("--frame-id", type=str, default="camera_init", help="Frame ID for initial pose")
    parser.add_argument("--wait-time", type=float, default=2.0, help="Time to wait before publishing (seconds)")
    
    parsed_args = parser.parse_args()
    
    # rclpy.init() cần None hoặc list of strings, không phải Namespace object
    rclpy.init(args=None)
    node = ExtractAndPublishInitialPose()
    
    # Wait a bit for ROS2 to be ready
    time.sleep(parsed_args.wait_time)
    
    # Extract pose from bag
    node.get_logger().info(f"Extracting initial pose from bag: {parsed_args.bag_path}")
    pose = None
    
    if HAS_ROSBAG2:
        pose = node.extract_first_pose_from_bag(parsed_args.bag_path, parsed_args.topic)
    else:
        node.get_logger().warn("rosbag2_py not available, using default pose")
    
    if pose:
        # Publish initial pose from bag
        node.get_logger().info("Publishing initial pose from bag...")
        if node.publish_initial_pose(pose, parsed_args.frame_id):
            node.get_logger().info("✅ Initial pose from bag published successfully")
            # Keep node alive for a bit to ensure message is sent
            time.sleep(1.0)
        else:
            node.get_logger().error("❌ Failed to publish initial pose")
            sys.exit(1)
    else:
        # DISABLED: Không publish default pose (0, 0, 0) vì sẽ gây drift
        # Hệ thống sẽ tự tìm vị trí bằng global localization (ScanContext + ICP)
        node.get_logger().info("ℹ️  Không publish initial pose - hệ thống sẽ tự tìm vị trí bằng global localization")
        node.get_logger().info("   (Pose trong /Odometry là odometry frame, không phải map frame)")
        # Không publish gì cả - để hệ thống tự tìm bằng global localization
        time.sleep(0.5)
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()

