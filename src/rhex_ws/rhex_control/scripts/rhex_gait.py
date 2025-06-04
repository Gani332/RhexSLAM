#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import time
import math

# PD gains
KP = 0.1
KD = 1.0

# Gait config
GAIT_FREQUENCY = 1.5   # Hz
STEP_AMPLITUDE = 1.0   # radians

ALL_JOINTS = [
    'front_left_leg_joint',
    'centre_left_leg_joint',
    'back_left_leg_joint',
    'front_right_leg_joint',
    'centre_right_leg_joint',
    'back_right_leg_joint'
]

TRIPOD_A = ['front_left_leg_joint', 'centre_right_leg_joint', 'back_left_leg_joint']
TRIPOD_B = ['front_right_leg_joint', 'centre_left_leg_joint', 'back_right_leg_joint']


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RHexCPGController(Node):
    def __init__(self):
        super().__init__('rhex_cpg_controller')

        self.publisher = self.create_publisher(Float64MultiArray, '/robot1/velocity_controller/commands', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/robot1/joint_states', self.joint_state_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.01, self.update)  # 100 Hz

        self.joint_angles = {j: 0.0 for j in ALL_JOINTS}
        self.joint_velocities = {j: 0.0 for j in ALL_JOINTS}

        self.start_time = time.time()
        self.forward = True
        self.moving = False

    def cmd_vel_callback(self, msg: Twist):
        if abs(msg.linear.x) > 0.1:
            self.moving = True
            self.forward = msg.linear.x > 0
        else:
            self.moving = False
            self.publish_stop()

    def joint_state_callback(self, msg: JointState):
        dt = 0.01  # 100 Hz
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_angles:
                vel = (pos - self.joint_angles[name]) / dt
                self.joint_velocities[name] = vel
                self.joint_angles[name] = pos

    def publish_stop(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * len(ALL_JOINTS)
        self.publisher.publish(msg)

    def update(self):
        if not self.moving:
            return

        t = time.time() - self.start_time
        freq = GAIT_FREQUENCY if self.forward else -GAIT_FREQUENCY
        omega = 2 * math.pi * freq
        commands = []

        for joint in ALL_JOINTS:
            if joint in TRIPOD_A:
                phase = 0.0
            else:
                phase = math.pi  # out of phase by 180°

            # Sinusoidal position trajectory for forward movement
            target_angle = STEP_AMPLITUDE * math.cos(omega * t + phase)

            error = normalize_angle(target_angle - self.joint_angles[joint])
            vel = self.joint_velocities[joint]
            cmd = KP * error - KD * vel
            commands.append(cmd)

        msg = Float64MultiArray()
        msg.data = commands
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RHexCPGController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
