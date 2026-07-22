#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class MecanumOdom(Node):

    def __init__(self):
        super().__init__('mecanum_odom')

        # Robot params
        self.r = 0.0439222
        self.wheelbase = 0.151410
        self.wheel_separation = 0.204108

        self.L = self.wheelbase / 2.0
        self.W = self.wheel_separation / 2.0

        # Pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Previous wheel positions/time
        self.prev_pos = None
        self.prev_time = None

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.get_logger().info("Mecanum odom node started")

    def joint_callback(self, msg):
        if len(msg.position) < 4:
            return

        current_time = self.get_clock().now()

        wheel_pos = dict(zip(msg.name, msg.position))

        try:
            p_lf = wheel_pos['left_f']
            p_rf = wheel_pos['right_f']
            p_lb = wheel_pos['left_b']
            p_rb = wheel_pos['right_b']
        except KeyError:
            self.get_logger().warn("Wheel names missing")
            return

        current_pos = [p_lf, p_rf, p_lb, p_rb]

        if self.prev_pos is None:
            self.prev_pos = current_pos
            self.prev_time = current_time
            return

        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        if dt <= 0:
            return

        # Angular wheel velocities (rad/s)
        w_lf = (current_pos[0] - self.prev_pos[0]) / dt
        w_rf = (current_pos[1] - self.prev_pos[1]) / dt
        w_lb = (current_pos[2] - self.prev_pos[2]) / dt
        w_rb = (current_pos[3] - self.prev_pos[3]) / dt

        self.prev_pos = current_pos
        self.prev_time = current_time

        # If rotation direction is wrong, uncomment:
        # w_rf = -w_rf
        # w_rb = -w_rb

        # Mecanum forward kinematics
        vx = self.r * (w_lf + w_rf + w_lb + w_rb) / 4.0
        vy = self.r * (-w_lf + w_rf + w_lb - w_rb) / 4.0
        wz = self.r * (-w_lf + w_rf - w_lb + w_rb) / (
            4.0 * (self.L + self.W)
        )

        # Robot frame -> world frame
        dx = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        dy = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        dyaw = wz * dt

        self.x += dx
        self.y += dy
        self.yaw += dyaw

        q = quaternion_from_euler(0, 0, self.yaw)

        # TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        # Odom
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        # Covariance
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.02

        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 0.01
        odom.twist.covariance[35] = 0.02

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()