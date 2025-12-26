#!/usr/bin/env python3

import copy
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import rclpy.timer
import tf2_geometry_msgs
import tf2_ros
# from geometry_msgs.msg import Transform - Not needed anymore
# from std_msgs.msg import Header - Not needed anymore


class TransformFusion(Node):
    def __init__(self):
        super().__init__("transform_fusion")

        self.cur_odom_to_baselink = None
        self.cur_map_to_odom = None

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.pub_localization = self.create_publisher(Odometry, "/localization", 1)

        self.create_subscription(Odometry, "/Odometry", self.cb_save_cur_odom, 1)
        self.create_subscription(Odometry, "/map_to_odom", self.cb_save_map_to_odom, 1)

        self.freq_pub_localization = 50
        self.timer = self.create_timer(1/self.freq_pub_localization, self.transform_fusion)
        # threading.Thread(target=self.transform_fusion, daemon=True).start()

    def pose_to_mat(self, pose_msg):
        trans = np.eye(4)
        trans[:3, 3] = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        quat = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
        # Convert quaternion to rotation matrix
        r = R.from_quat(quat)
        trans[:3, :3] = r.as_matrix()
        return trans

    def transform_fusion(self):
        if self.cur_odom_to_baselink is None:
            return

        if self.cur_map_to_odom is not None:
            T_map_to_odom = self.pose_to_mat(self.cur_map_to_odom.pose.pose)
        else:
            T_map_to_odom = np.eye(4)

        # No need to publish TF here - FAST_LIO handles camera_init → body
        # This node only publishes localization odometry

        cur_odom = copy.copy(self.cur_odom_to_baselink)
        if cur_odom is not None:
            T_odom_to_base_link = self.pose_to_mat(cur_odom.pose.pose)
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)

            xyz = T_map_to_base_link[:3, 3]
            r = R.from_matrix(T_map_to_base_link[:3, :3])
            quat = r.as_quat()  # Returns [x, y, z, w]

            localization = Odometry()
            localization.pose.pose = Pose(
                position = Point(x = xyz[0], y = xyz[1], z = xyz[2]), 
                orientation = Quaternion(x = quat[0], y = quat[1], z = quat[2], w = quat[3])
            )
            localization.twist = cur_odom.twist

            localization.header.stamp = self.get_clock().now().to_msg()
            localization.header.frame_id = "camera_init"
            localization.child_frame_id = "body"
            self.pub_localization.publish(localization)


    def cb_save_cur_odom(self, msg):
        self.cur_odom_to_baselink = msg

    def cb_save_map_to_odom(self, msg):
        self.cur_map_to_odom = msg


def main(args=None):
    rclpy.init(args=args)
    node = TransformFusion()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
