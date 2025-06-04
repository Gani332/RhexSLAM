import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ThermalViewer(Node):
    def __init__(self):
        super().__init__('thermal_viewer_node')
        self.subscription = self.create_subscription(
            Image,
            '/thermal/image_raw',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        cv2.imshow("Thermal Image", cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ThermalViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
