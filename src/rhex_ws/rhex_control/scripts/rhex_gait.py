#!/usr/bin/env python3
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

FREQUENCY = 200.0
MAX_SPEED = 5.0
SCALE_LINEAR = 1.0
SCALE_ANGULAR = 2.0
STEP_THRESHOLD = 6.28  # radians per step (≈1 rotation)

TRIPOD_A = ['front_left_leg_joint', 'centre_right_leg_joint', 'back_left_leg_joint']
TRIPOD_B = ['front_right_leg_joint', 'centre_left_leg_joint', 'back_right_leg_joint']
ALL_JOINTS = TRIPOD_A + TRIPOD_B

class RHexCmdVelTripodController(Node):
    def __init__(self):
        super().__init__('rhex_cmdvel_tripod_controller')

        self.publisher = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(1.0 / FREQUENCY, self.update)

        self.joint_order = ALL_JOINTS
        self.current_tripod = TRIPOD_A
        self.waiting_tripod = TRIPOD_B

        self.linear_x = 0.0
        self.angular_z = 0.0

        self.joint_angles = {joint: 0.0 for joint in self.joint_order}
        self.last_joint_angles = self.joint_angles.copy()
        self.joint_velocities = {joint: 0.0 for joint in self.joint_order}
        self.tripod_start_angles = {joint: 0.0 for joint in self.current_tripod}

        self.last_debug_log = 0
        self.get_logger().info("RHex cmd_vel tripod gait controller (predictive stepping) started.")

    def cmd_vel_callback(self, msg: Twist):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def joint_state_callback(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            self.get_logger().warn("JointState message has mismatched name and position lengths.")
            return

        now = self.get_clock().now().seconds_nanoseconds()[0]
        updated = {}

        for name, position in zip(msg.name, msg.position):
            if name in self.joint_angles:
                last_pos = self.joint_angles[name]
                self.last_joint_angles[name] = last_pos
                self.joint_angles[name] = position
                self.joint_velocities[name] = (position - last_pos) * FREQUENCY
                updated[name] = position

        if now - self.last_debug_log >= 1:
            self.get_logger().info(f"Updated Joint Angles: {updated}")
            self.last_debug_log = now

    def update(self):
        velocities = []

        if self.linear_x == 0.0 and self.angular_z == 0.0:
            velocities = [0.0 for _ in self.joint_order]
        else:
            stepped = False
            for joint in self.current_tripod:
                current = self.joint_angles[joint]
                velocity = self.joint_velocities[joint]
                predicted = current + velocity / FREQUENCY
                diff = abs(predicted - self.tripod_start_angles[joint])
                if diff >= STEP_THRESHOLD:
                    stepped = True
                    break

            if stepped:
                self.current_tripod, self.waiting_tripod = self.waiting_tripod, self.current_tripod
                for j in self.current_tripod:
                    self.tripod_start_angles[j] = self.joint_angles[j]
                self.get_logger().info(f"Switched tripod: now moving {self.current_tripod}")

            left_speed = (self.linear_x - self.angular_z * SCALE_ANGULAR) * SCALE_LINEAR
            right_speed = (self.linear_x + self.angular_z * SCALE_ANGULAR) * SCALE_LINEAR
            left_speed = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
            right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)

            for joint in self.joint_order:
                if joint in self.current_tripod:
                    if 'left' in joint:
                        velocities.append(left_speed)
                    elif 'right' in joint:
                        velocities.append(right_speed)
                    else:
                        velocities.append(self.linear_x)
                else:
                    velocities.append(0.0)

        msg = Float64MultiArray()
        msg.data = velocities
        self.publisher.publish(msg)

        now = self.get_clock().now().seconds_nanoseconds()[0]
        if now - self.last_debug_log >= 1:
            self.get_logger().info(f"Publishing velocities: {velocities}")
            self.last_debug_log = now

def main(args=None):
    rclpy.init(args=args)
    node = RHexCmdVelTripodController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
