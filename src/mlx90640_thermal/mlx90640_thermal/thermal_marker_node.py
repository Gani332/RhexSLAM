import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion
from tf2_ros import TransformListener, Buffer
import tf2_ros
import math
import time

class ThermalMarkerNode(Node):
    def __init__(self):
        super().__init__('thermal_marker_node')
        self.sub = self.create_subscription(Bool, '/survivor_detected', self.callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_scan = None
        self.frame_id = 'laser'   # LiDAR frame (same as base_link here)
        self.map_frame = 'map'

        self.last_detection_time = 0
        self.cooldown = 3  # seconds between markers

    def scan_callback(self, msg):
        self.last_scan = msg

    def callback(self, msg):
        if not msg.data or self.last_scan is None:
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.frame_id,
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y
        rot = trans.transform.rotation
        _, _, yaw = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

        # Find the closest scan in ±10 degrees (approx. ±0.17 rad)
        angle_min = self.last_scan.angle_min
        angle_increment = self.last_scan.angle_increment
        ranges = self.last_scan.ranges

        best_range = None
        best_angle = None
        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            angle = angle_min + i * angle_increment
            if abs(angle) < 0.17:  # narrow forward cone
                if best_range is None or r < best_range:
                    best_range = r
                    best_angle = angle

        if best_range is None:
            self.get_logger().info("No valid obstacle found in forward direction.")
            return

        # Compute obstacle position in robot frame
        local_x = math.cos(best_angle) * best_range
        local_y = math.sin(best_angle) * best_range

        # Transform to map frame
        marker_x = x + math.cos(yaw) * local_x - math.sin(yaw) * local_y
        marker_y = y + math.sin(yaw) * local_x + math.cos(yaw) * local_y

        now_sec = time.time()
        if now_sec - self.last_detection_time < self.cooldown:
            return

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = marker_x
        marker.pose.position.y = marker_y
        marker.pose.position.z = 0.1
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.4
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.id = int(now_sec)
        marker.lifetime.sec = 30

        self.marker_pub.publish(marker)
        self.get_logger().info(f"📍 Marker placed at obstacle. Distance: {best_range:.2f}m")
        self.last_detection_time = now_sec

def main(args=None):
    rclpy.init(args=args)
    node = ThermalMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
