#!/usr/bin/env python3

from auto_patrol import patrol
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import time


class Patrol(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self.navigator = BasicNavigator()

    def create_pose(self, x, y, yaw=0.0):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        return goal


def main():
    rclpy.init()
    patrol = Patrol()

    navigator = patrol.navigator

    navigator.waitUntilNav2Active()

    waypoints = [
        patrol.create_pose(-1.0, 3.0),
        patrol.create_pose(-3.0, 3.0),
        patrol.create_pose(-3.0, 1.0),
        patrol.create_pose(-1.0, 1.0),
    ]
    figure8 = [
        patrol.create_pose(-2.8, 2.0),
        patrol.create_pose(-2.6, 2.6),
        patrol.create_pose(-2.0, 2.8),
        patrol.create_pose(-1.4, 2.6),
        patrol.create_pose(-1.2, 2.0),
        patrol.create_pose(-1.4, 1.4),
        patrol.create_pose(-2.0, 1.2),
        patrol.create_pose(-2.6, 1.4),

        patrol.create_pose(-2.0, 2.0),  # crossover

        patrol.create_pose(-1.2, 2.0),
        patrol.create_pose(-0.6, 2.6),
        patrol.create_pose(0.0, 2.8),
        patrol.create_pose(0.6, 2.6),
        patrol.create_pose(0.8, 2.0),
        patrol.create_pose(0.6, 1.4),
        patrol.create_pose(0.0, 1.2),
        patrol.create_pose(-0.6, 1.4),

        patrol.create_pose(-2.0, 2.0)
    ]
    while rclpy.ok():
        for wp in waypoints:
            navigator.goToPose(wp)

            while not navigator.isTaskComplete():
                time.sleep(0.5)

            print("Reached square waypoint")

    while rclpy.ok():
        for wp in figure8:
            navigator.goToPose(wp)
            while not navigator.isTaskComplete():
                time.sleep(0.3)
            print("Reached figure8 waypoint")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
