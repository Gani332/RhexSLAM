import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ThermalViewer(Node):
    def __init__(self):
        super().__init__('thermal_viewer_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/thermal/image_raw',
            self.listener_callback,
            10
        )
        self.get_logger().info("ThermalViewer node started. Waiting for images...")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            cv2.imshow("Thermal Image", cv_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rclpy.shutdown()
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unhandled error: {e}")

def main():
    rclpy.init()
    node = ThermalViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
