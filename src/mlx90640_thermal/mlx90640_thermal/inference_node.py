import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

# Human detection thresholds
TEMP_MIN = 32.0
TEMP_MAX = 39.0
TEMP_HARD_CAP = 40.0

BLOB_AREA_MIN = 2
BLOB_AREA_MAX = 1000

class ThermalInference(Node):
    def __init__(self):
        super().__init__('thermal_inference_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/thermal/image_raw',
            self.callback,
            10
        )
        self.publisher = self.create_publisher(Bool, '/survivor_detected', 10)

    def callback(self, msg):
        try:
            raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            temps = (raw_image.astype(np.float32) / 255.0) * (40 - 20) + 20

            min_temp = np.min(temps)
            max_temp = np.max(temps)
            self.get_logger().info(f"Frame: min={min_temp:.1f}°C, max={max_temp:.1f}°C")

            if max_temp >= TEMP_HARD_CAP:
                self.get_logger().warn(f"❌ Frame rejected: max temp {max_temp:.1f}°C too high")
                self.publisher.publish(Bool(data=False))
                return

            # Count pixels in human range
            human_mask = np.logical_and(temps >= TEMP_MIN, temps <= TEMP_MAX)
            human_pixel_count = np.sum(human_mask)
            #self.get_logger().info(f"Pixels in human temp range: {human_pixel_count}")

            # Resize and threshold for optional contour check
            norm_temps = np.clip((temps - TEMP_MIN) / (TEMP_MAX - TEMP_MIN), 0, 1)
            norm_image = (norm_temps * 255).astype(np.uint8)
            resized = cv2.resize(norm_image, (320, 240), interpolation=cv2.INTER_NEAREST)
            _, thresh = cv2.threshold(resized, 127, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # If enough pixels are warm, accept
            if human_pixel_count >= 25:
                self.get_logger().info("✅ Human detected based on pixel count")
                self.publisher.publish(Bool(data=True))
                return

            # Fallback: try to detect human-like blob
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if BLOB_AREA_MIN < area < BLOB_AREA_MAX:
                    self.get_logger().info(f"✅ Detected human-like blob: area = {area:.1f}")
                    self.publisher.publish(Bool(data=True))
                    return

            self.get_logger().info("❌ No human detected.")
            self.publisher.publish(Bool(data=False))

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")
            self.publisher.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = ThermalInference()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
