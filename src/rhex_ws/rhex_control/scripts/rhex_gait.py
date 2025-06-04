#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import time
import math

# Control gains
KP = 0.2
KD = 1.0

# Gait parameters
STEP_ANGLE = 1.2  # radians (~70 degrees)
FREQUENCY = 100   # Hz control loop
STEP_TOLERANCE = 0.2  # radians

# Joint definitions
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
    """Normalize angle to [-π, π]"""
    return math.atan2(math.sin(angle), math.cos(angle))


class RHexStepper(Node):
    def __init__(self):
        super().__init__('rhex_stepper')

        self.publisher = self.create_publisher(Float64MultiArray, '/robot1/velocity_controller/commands', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/robot1/joint_states', self.joint_state_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(1.0 / FREQUENCY, self.update)

        self.joint_angles = {j: 0.0 for j in ALL_JOINTS}
        self.joint_velocities = {j: 0.0 for j in ALL_JOINTS}

        self.active_tripod = TRIPOD_A
        self.waiting_tripod = TRIPOD_B
        self.leg_targets = {j: 0.0 for j in ALL_JOINTS}
        self.leg_done = {j: False for j in ALL_JOINTS}
        self.step_index = {j: 0 for j in ALL_JOINTS}

        self.moving = False
        self.step_direction = 1.0  # forward

    def cmd_vel_callback(self, msg: Twist):
        if abs(msg.linear.x) > 0.1:
            self.moving = True
            self.step_direction = 1.0 if msg.linear.x > 0 else -1.0
        else:
            self.moving = False
            self.publish_stop()

    def joint_state_callback(self, msg: JointState):
        dt = 1.0 / FREQUENCY
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_angles:
                vel = (pos - self.joint_angles[name]) / dt
                self.joint_velocities[name] = vel
                self.joint_angles[name] = pos

    def publish_stop(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * len(ALL_JOINTS)
        self.publisher.publish(msg)

    def start_step(self):
        for leg in self.active_tripod:
            self.step_index[leg] += 1
            self.leg_targets[leg] = self.step_index[leg] * self.step_direction * STEP_ANGLE
            self.leg_done[leg] = False
        self.get_logger().info(f"Stepping tripod: {self.active_tripod}")

    def update(self):
        if not self.moving:
            return

        # If all legs in the active tripod are done, switch tripods
        if all(self.leg_done[leg] for leg in self.active_tripod):
            self.active_tripod, self.waiting_tripod = self.waiting_tripod, self.active_tripod
            self.start_step()

        commands = []
        for j in ALL_JOINTS:
            if j in self.active_tripod and not self.leg_done[j]:
                target = self.leg_targets[j]
                error = normalize_angle(target - self.joint_angles[j])
                vel = self.joint_velocities[j]
                cmd = KP * error - KD * vel
                commands.append(cmd)

                if abs(error) < STEP_TOLERANCE:
                    self.leg_done[j] = True
            else:
                commands.append(0.0)

        msg = Float64MultiArray()
        msg.data = commands
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RHexStepper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
