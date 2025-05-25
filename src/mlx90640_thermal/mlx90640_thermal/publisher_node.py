import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import board
import busio
import adafruit_mlx90640
import cv2

class ThermalPublisher(Node):
    def __init__(self):
        super().__init__('thermal_publisher_node')
        self.publisher_ = self.create_publisher(Image, '/thermal/image_raw', 10)
        self.bridge = CvBridge()

        # Sensor setup
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.mlx = adafruit_mlx90640.MLX90640(self.i2c)
        self.mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ

        self.timer = self.create_timer(0.5, self.publish_thermal)

    def publish_thermal(self):
        frame = np.zeros((24 * 32,))
        self.mlx.getFrame(frame)
        temps = np.reshape(frame, (24, 32))

        norm = np.clip((temps - 20) / (40 - 20), 0, 1) * 255
        image = norm.astype(np.uint8)
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="mono8")
        self.publisher_.publish(ros_image)

        # Add this log line
        self.get_logger().info("Published thermal frame")


def main(args=None):
    rclpy.init(args=args)
    node = ThermalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
