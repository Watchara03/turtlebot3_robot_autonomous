#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import threading
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

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

        self.navigator = BasicNavigator()

        self.timer = self.create_timer(1.0, self.check_for_amcl_or_odom)
        self.amcl_update_count = 0
        self.nav_count = 0

    def odom_callback(self, msg):
        self.odom_msg = msg
        self.odom_event.set()

    def amcl_pose_callback(self, msg):
        self.amcl_pose_msg = msg
        self.amcl_pose_event.set()

    def check_for_amcl_or_odom(self):
        if self.amcl_pose_msg :
            self.set_initial_pose_from_amcl()
        elif self.odom_msg  :
            self.set_initial_pose_from_odom()
        else:
            self.get_logger().info('Waiting for AMCL or odom messages...')

    def set_initial_pose_from_amcl(self):
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
        self.get_logger().info('Setting initial pose from amcl')
        self.publisher.publish(init_msg)
        self.get_logger().info('Initial pose set from amcl')

        # Start listening for continuous updates from AMCL
        self.timer.cancel()
        self.timer = self.create_timer(0.5, self.update_pose_from_amcl)

    def set_initial_pose_from_odom(self):
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
        init_msg.pose.covariance[0] = 0.1  # x covariance
        init_msg.pose.covariance[7] = 0.1  # y covariance
        init_msg.pose.covariance[35] = 0.68  # yaw covariance

        # Publish the initial pose from odom
        self.get_logger().info('Setting initial pose from odom')
        self.publisher.publish(init_msg)
        self.get_logger().info('Initial pose set from odom')

        # Start listening for continuous updates from AMCL
        self.timer.cancel()
        self.timer = self.create_timer(0.5, self.update_pose_from_amcl)

    def update_pose_from_amcl(self):
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

        # Increment amcl_update_count
        self.amcl_update_count += 1

        # Check if it's time to stop AMCL updates
        if self.amcl_update_count >= 25:
            self.stop_amcl_update()
            self.execute_navigation_task()
        else:
            # Publish the updated pose from amcl_pose
            self.get_logger().info('Updating pose from amcl_pose')
            self.publisher.publish(init_msg)
            self.get_logger().info('Pose updated from amcl_pose')

    def stop_amcl_update(self):
        # Stop AMCL update timer
        self.timer.cancel()

    def number_nav(self): 
        self.nav_count += 1
        if self.nav_count == 1 :
            self.execute_navigation_task()

            return
        elif self.nav_count == 2 :
            self.execute_navigation_task()

    def execute_navigation_task2(self):
        # Add your navigation task code here
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.1579
        goal_pose.pose.position.y = -0.04159
        goal_pose.pose.orientation.w = 0.694

        # Go to the goal pose
        self.navigator.goToPose(goal_pose)

        # Wait until the task is complete
        # while not self.navigator.isTaskComplete():
        #     pass

        # Get the result of the navigation task
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Navigation goal succeeded! ___2___')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Navigation goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Navigation goal failed!')
        else:
            self.get_logger().info('Navigation goal has an invalid return status!')

    def execute_navigation_task(self):
        # Add your navigation task code here
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.783
        goal_pose.pose.position.y = 0.028
        goal_pose.pose.orientation.w = 0.7
        # Go to the goal pose
        self.navigator.goToPose(goal_pose)

        # Wait until the task is complete
        # while not self.navigator.isTaskComplete():
        #     pass

        # Get the result of the navigation task
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Navigation goal succeeded!  ___1___')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Navigation goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Navigation goal failed!')
        else:
            self.get_logger().info('Navigation goal has an invalid return status!')

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
