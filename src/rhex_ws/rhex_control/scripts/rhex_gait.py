#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

FREQUENCY = 100.0
STEP_SIZE = 1.5        # radians to move each step
STEP_THRESHOLD = 6.28  # radians per stride (e.g., 1 full rotation)

TRIPOD_A = ['front_left_leg_joint', 'centre_right_leg_joint', 'back_left_leg_joint']
TRIPOD_B = ['front_right_leg_joint', 'centre_left_leg_joint', 'back_right_leg_joint']
ALL_JOINTS = TRIPOD_A + TRIPOD_B

class PIDController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd

    def compute(self, error, velocity):
        return self.kp * error - self.kd * velocity

class RHexTripodPIDController(Node):
    def __init__(self):
        super().__init__('rhex_tripod_pid_controller')

        self.publisher = self.create_publisher(Float64MultiArray, '/robot1/velocity_controller/commands', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/robot1/joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(1.0 / FREQUENCY, self.update)

        # Motion state
        self.linear_x = 0.0
        self.angular_z = 0.0

        # Tripod state
        self.current_tripod = TRIPOD_A
        self.waiting_tripod = TRIPOD_B

        # Joint state tracking
        self.joint_angles = {j: 0.0 for j in ALL_JOINTS}
        self.joint_velocities = {j: 0.0 for j in ALL_JOINTS}
        self.target_angles = {j: 0.0 for j in ALL_JOINTS}
        self.hold_position = {j: 0.0 for j in ALL_JOINTS}

        # Anchoring stride progress
        self.initial_angles = {}
        self.last_stride_index = {}

        # PID per joint
        self.pid = {j: PIDController(kp=4.0, kd=1.0) for j in ALL_JOINTS}

        self.get_logger().info("RHex tripod PID gait controller started.")

    def cmd_vel_callback(self, msg: Twist):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def joint_state_callback(self, msg: JointState):
        dt = 1.0 / FREQUENCY
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_angles:
                vel = (pos - self.joint_angles[name]) / dt
                self.joint_velocities[name] = vel
                self.joint_angles[name] = pos

    def update(self):
        if not self.initial_angles and any(self.joint_angles.values()):
            self.initial_angles = {j: self.joint_angles[j] for j in ALL_JOINTS}
            self.last_stride_index = {j: 0 for j in ALL_JOINTS}
            self.get_logger().info("Initial joint angles recorded.")

        if not self.initial_angles:
            return  # still waiting for valid data

        # Trigger first step
        if self.linear_x != 0.0 and all(v == 0.0 for v in self.target_angles.values()):
            step_direction = STEP_SIZE if self.linear_x >= 0 else -STEP_SIZE
            for j in self.current_tripod:
                self.target_angles[j] = self.joint_angles[j] + step_direction
            for j in self.waiting_tripod:
                self.hold_position[j] = self.joint_angles[j]
            self.get_logger().info("Initialized first step.")

        # Get stride index of center leg relative to initial
        direction = 1 if self.linear_x >= 0 else -1

        for j in self.current_tripod:
            if "centre" not in j:
                continue
            moved = self.joint_angles[j] - self.initial_angles[j]
            current_stride = int(math.floor(abs(moved) / STEP_THRESHOLD)) * direction

            # Switch phase if new stride index is greater
            if current_stride != self.last_stride_index[j]:
                self.last_stride_index[j] = current_stride
                self.current_tripod, self.waiting_tripod = self.waiting_tripod, self.current_tripod

                step_direction = STEP_SIZE if self.linear_x >= 0 else -STEP_SIZE
                for k in self.current_tripod:
                    self.target_angles[k] = self.joint_angles[k] + step_direction
                for k in self.waiting_tripod:
                    self.hold_position[k] = self.joint_angles[k]

                self.get_logger().info(f"Switched tripod: {self.current_tripod}")
                break  # only switch once per update

        # Compute PID control
        commands = []
        for j in ALL_JOINTS:
            if j in self.current_tripod:
                error = self.target_angles[j] - self.joint_angles[j]
                velocity = self.joint_velocities[j]
                cmd = self.pid[j].compute(error, velocity)
                commands.append(cmd)
            else:
                commands.append(0.0)

        msg = Float64MultiArray()
        msg.data = commands
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RHexTripodPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
