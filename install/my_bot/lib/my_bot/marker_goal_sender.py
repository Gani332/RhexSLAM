#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

class MarkerGoalSender(Node):
    def __init__(self):
        super().__init__('marker_goal_sender')
        self.marker_sub = self.create_subscription(Marker, '/visualization_marker', self.marker_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal_sent = False

    def marker_callback(self, msg):
        if self.goal_sent:
            return
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available!")
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = msg.pose  # Use marker pose directly

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f"🚀 Sending goal to ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})")
        self.nav_client.send_goal_async(goal_msg)
        self.goal_sent = True

def main(args=None):
    rclpy.init(args=args)
    node = MarkerGoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
