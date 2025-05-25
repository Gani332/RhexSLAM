#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial

class EncoderReader(Node):
    def __init__(self):
        super().__init__('encoder_reader_node')

        # Serial connection to Arduino/GIGA (update port if needed)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

        # Publisher for joint states
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.read_serial)  # 20 Hz

        # Joint names as defined in URDF
        self.joint_names = [
            'front_left_leg_joint', 'centre_left_leg_joint', 'back_left_leg_joint',
            'front_right_leg_joint', 'centre_right_leg_joint', 'back_right_leg_joint'
        ]

        # Mapping from serial labels to URDF joint names
        self.name_mapping = {
            'l1': 'front_left_leg_joint',
            'l2': 'centre_left_leg_joint',
            'l3': 'back_left_leg_joint',
            'r1': 'front_right_leg_joint',
            'r2': 'centre_right_leg_joint',
            'r3': 'back_right_leg_joint'
        }

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line.endswith(';'):
                return
            line = line[:-1]  # Remove trailing ;

            values = {}
            for pair in line.split(','):
                name, val = pair.split(':')
                if name in self.name_mapping:
                    joint_name = self.name_mapping[name]
                    values[joint_name] = float(val)

            if len(values) != 6:
                return

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [values[name] for name in self.joint_names]

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# serial_line = "front_left_leg_joint:2.5,centre_left_leg_joint:2.5,back_left_leg_joint:2.5," \
#               "front_right_leg_joint:3.0,centre_right_leg_joint:3.0,back_right_leg_joint:3.0;"
