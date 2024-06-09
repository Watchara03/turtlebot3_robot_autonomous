#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import time

"""
Basic security route patrol demo. In this demonstration, the expectation
is that there are security cameras mounted on the robots recording or being
watched live by security staff.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Security route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    security_route = [
        [0.794, -0.083,  0.704],  # Camera 
        [0.785, -0.580, -0.710], # position 1
        [0.179, -0.580, -0.750], # position 2
        [0.097, -0.125, -1.851], # Home
    ]

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -0.008
    initial_pose.pose.position.y = -0.090
    initial_pose.pose.orientation.z = -0.007
    initial_pose.pose.orientation.w = 0.999
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    count = 0

    while rclpy.ok():
        for pt in security_route:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = navigator.get_clock().now().to_msg()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.orientation.z = pt[2]

            # Send the robot to the next position
            navigator.goToPose(pose)

            # Wait until the task is complete
            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback:
                    print('Estimated time to complete current route: ' + '{0:.0f}'.format(
                          Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                          + ' seconds.')

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f'Arrived at position: {pt}')
                time.sleep(5)  # Wait for 2 seconds
                count += 1
                print("Completed route cycle count:", count)
                if count == 4:
                    exit(1)
            elif result == TaskResult.CANCELED:
                print('Navigation task was canceled, exiting.')
                exit(1)
            elif result == TaskResult.FAILED:
                print('Navigation task failed, restarting...')
                break  # Exit the current for loop to restart the route

if __name__ == '__main__':
    main()
