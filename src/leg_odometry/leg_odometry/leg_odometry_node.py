#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class LegOdometryNode(Node):
    def __init__(self):
        super().__init__('leg_odometry_node')

        self.joint_names = [
            'front_left_leg_joint', 'centre_left_leg_joint', 'back_left_leg_joint',
            'front_right_leg_joint', 'centre_right_leg_joint', 'back_right_leg_joint'
        ]

        self.left_legs = self.joint_names[:3]
        self.right_legs = self.joint_names[3:]

        self.prev_angles = dict.fromkeys(self.joint_names, 0.0)
        self.prev_time = self.get_clock().now().nanoseconds / 1e9

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Robot geometry
        self.r = 0.03       # leg radius (m)
        self.alpha = 0.0    # leg extension angle (rad)
        self.lc = 0.02      # chassis height from pivot (m)
        self.wheelbase_width = 0.25  # distance between left and right legs (m)

        # Precomputed limits for contact phase
        self.theta_start = math.radians(90 - 2 * math.degrees(math.acos(math.sqrt(self.lc / (2 * self.r)))))
        self.theta_end = self.alpha + math.acos(self.lc / (2 * self.r * math.cos(self.alpha)))

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom/leg', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def joint_state_callback(self, msg: JointState):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.prev_time
        if dt <= 0:
            return

        dx_total = 0.0
        dy_total = 0.0

        dx_per_leg = {}

        for name, angle in zip(msg.name, msg.position):
            if name not in self.joint_names:
                continue

            angle = (angle + math.pi) % (2 * math.pi) - math.pi
            prev_angle = self.prev_angles[name]
            prev_angle = (prev_angle + math.pi) % (2 * math.pi) - math.pi

            dtheta = angle - prev_angle

            if self.theta_start <= angle <= 2 * self.alpha:
                dx = self.r * (dtheta + math.sin(angle) - math.sin(prev_angle))
                dy = self.r * (math.cos(angle) - math.cos(prev_angle))
            elif 2 * self.alpha < angle <= self.theta_end:
                dx = 2 * self.r * (math.sin(angle) - math.sin(prev_angle))
                dy = 2 * self.r * (math.cos(angle) - math.cos(prev_angle))
            else:
                dx = 0.0
                dy = 0.0


            dx_total += dx
            dy_total += dy
            dx_per_leg[name] = dx

            self.prev_angles[name] = angle

        left_dx = sum(dx_per_leg.get(j, 0.0) for j in self.left_legs)
        right_dx = sum(dx_per_leg.get(j, 0.0) for j in self.right_legs)
        dtheta = (right_dx - left_dx) / self.wheelbase_width
        self.theta += dtheta

        dx_global = dx_total * math.cos(self.theta) - dy_total * math.sin(self.theta)
        dy_global = dx_total * math.sin(self.theta) + dy_total * math.cos(self.theta)

        self.x += dx_global
        self.y += dy_global

        self.prev_time = now
        self.publish_odometry()
        self.publish_transform()

    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.odom_pub.publish(odom_msg)

    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = LegOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
