#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import threading
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class InitPoseNode(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('init_pose')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(1.0, self.execute_navigation_task)

        navigator = BasicNavigator()

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.034
        initial_pose.pose.position.y = -0.116
        initial_pose.pose.orientation.z = -0.005
        initial_pose.pose.orientation.w = 0.999
        navigator.setInitialPose(initial_pose)

        navigator.waitUntilNav2Active()

    def execute_navigation_task(self):
        navigator = BasicNavigator()
        # Add your navigation task code here
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.783
        goal_pose.pose.position.y = 0.028
        goal_pose.pose.orientation.w = 0.85
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
