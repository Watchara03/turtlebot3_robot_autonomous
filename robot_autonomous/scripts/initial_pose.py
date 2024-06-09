#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import threading

class InitPoseNode(Node):
    def __init__(self):
        super().__init__('init_pose')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.odom_msg = None
        self.amcl_pose_msg = None
        self.odom_event = threading.Event()
        self.amcl_pose_event = threading.Event()

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)

        self.timer = self.create_timer(1.0, self.set_initial_pose_from_odom)

    def odom_callback(self, msg):
        self.odom_msg = msg
        self.odom_event.set()

    def amcl_pose_callback(self, msg):
        self.amcl_pose_msg = msg
        self.amcl_pose_event.set()

    def set_initial_pose_from_odom(self):
        self.odom_event.wait()

        if self.odom_msg is None:
            self.get_logger().error('No odom message received')
            return

        init_msg = PoseWithCovarianceStamped()
        init_msg.header.frame_id = 'map'

        # Set initial pose from odom
        init_msg.pose.pose.position.x = self.odom_msg.pose.pose.position.x
        init_msg.pose.pose.position.y = self.odom_msg.pose.pose.position.y
        init_msg.pose.pose.orientation.x = self.odom_msg.pose.pose.orientation.x
        init_msg.pose.pose.orientation.y = self.odom_msg.pose.pose.orientation.y
        init_msg.pose.pose.orientation.z = self.odom_msg.pose.pose.orientation.z
        init_msg.pose.pose.orientation.w = self.odom_msg.pose.pose.orientation.w

        # Set default covariance values for initial pose
        init_msg.pose.covariance[0] = 0.5  # x covariance
        init_msg.pose.covariance[7] = 0.5  # y covariance
        init_msg.pose.covariance[35] = 1.5  # yaw covariance

                # Publish the initial pose from odom
        self.get_logger().info('Setting initial pose from odom')
        self.publisher.publish(init_msg)
        self.get_logger().info('Initial pose set from odom')

        # Switch to listening for amcl_pose updates
        self.timer.cancel()
        self.timer = self.create_timer(0.3, self.update_pose_from_amcl)

    def update_pose_from_amcl(self):
        self.amcl_pose_event.wait()

        if self.amcl_pose_msg is None:
            self.get_logger().error('No amcl_pose message received')
            return

        init_msg = PoseWithCovarianceStamped()
        init_msg.header.frame_id = 'map'

        # Update pose from amcl_pose
        init_msg.pose.pose.position.x = self.amcl_pose_msg.pose.pose.position.x
        init_msg.pose.pose.position.y = self.amcl_pose_msg.pose.pose.position.y
        init_msg.pose.pose.orientation.x = self.amcl_pose_msg.pose.pose.orientation.x
        init_msg.pose.pose.orientation.y = self.amcl_pose_msg.pose.pose.orientation.y
        init_msg.pose.pose.orientation.z = self.amcl_pose_msg.pose.pose.orientation.z
        init_msg.pose.pose.orientation.w = self.amcl_pose_msg.pose.pose.orientation.w

        # Update covariance from amcl_pose
        init_msg.pose.covariance = self.amcl_pose_msg.pose.covariance

        # Publish the updated pose from amcl_pose
        self.get_logger().info('Updating pose from amcl_pose')
        self.publisher.publish(init_msg)
        self.get_logger().info('Pose updated from amcl_pose')

def main(args=None):
    rclpy.init(args=args)
    node = InitPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

