#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

FREQUENCY = 100.0
STEP_DURATION = 1.0  # seconds each tripod is active
MAX_SPEED = 5.0      # max motor speed for mapping cmd_vel
SCALE_LINEAR = 1.0   # scales how much linear.x affects leg speed
SCALE_ANGULAR = 2.0  # scales how much angular.z affects left/right tripod balance

TRIPOD_A = ['front_left_leg_joint', 'centre_right_leg_joint', 'back_left_leg_joint']
TRIPOD_B = ['front_right_leg_joint', 'centre_left_leg_joint', 'back_right_leg_joint']
ALL_JOINTS = TRIPOD_A + TRIPOD_B

class RHexCmdVelTripodController(Node):
    def __init__(self):
        super().__init__('rhex_cmdvel_tripod_controller')

        self.publisher = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(1.0 / FREQUENCY, self.update)

        self.joint_order = ALL_JOINTS
        self.current_tripod = TRIPOD_A
        self.waiting_tripod = TRIPOD_B
        self.elapsed = 0.0

        # Most recent velocity command
        self.linear_x = 0.0
        self.angular_z = 0.0

        self.get_logger().info("RHex cmd_vel tripod gait controller started.")

    def cmd_vel_callback(self, msg: Twist):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def update(self):
        velocities = []

        if self.linear_x == 0.0 and self.angular_z == 0.0:
            # Stop all motors if no command
            velocities = [0.0 for _ in self.joint_order]
        else:
            self.elapsed += 1.0 / FREQUENCY
            if self.elapsed >= STEP_DURATION:
                self.elapsed = 0.0
                self.current_tripod, self.waiting_tripod = self.waiting_tripod, self.current_tripod
                self.get_logger().info(f"Switched tripod: now moving {self.current_tripod}")

            # Map linear and angular velocity into motor speeds
            left_speed = (self.linear_x - self.angular_z * SCALE_ANGULAR) * SCALE_LINEAR
            right_speed = (self.linear_x + self.angular_z * SCALE_ANGULAR) * SCALE_LINEAR

            # Clamp
            left_speed = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
            right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)

            # Assign speeds to current tripod
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

def main(args=None):
    rclpy.init(args=args)
    node = RHexCmdVelTripodController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
