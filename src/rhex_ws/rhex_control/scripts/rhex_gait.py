#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

FREQUENCY = 100.0
STEP_SIZE = 1.5  # radians, each step target
STEP_TIME = 0.5  # minimum phase time in seconds

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
        self.hold_position = {j: 0.0 for j in ALL_JOINTS}

        self.publisher = self.create_publisher(Float64MultiArray, '/robot1/velocity_controller/commands', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/robot1/joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(1.0 / FREQUENCY, self.update)

        # Motion state
        self.linear_x = 0.0
        self.angular_z = 0.0

        # Timing for tripod switch
        self.phase_start_time = self.get_clock().now().nanoseconds / 1e9

        # Tripod state
        self.current_tripod = TRIPOD_A
        self.waiting_tripod = TRIPOD_B

        # Joint tracking
        self.joint_angles = {j: 0.0 for j in ALL_JOINTS}
        self.joint_velocities = {j: 0.0 for j in ALL_JOINTS}
        self.target_angles = {j: 0.0 for j in ALL_JOINTS}

        # PID per joint
        self.motion_pid = PIDController(kp=1.0, kd=0.0)
        self.hold_pid = PIDController(kp=5.0, kd=0.2)
        

        self.get_logger().info("RHex tripod PID gait controller started.")

    def cmd_vel_callback(self, msg: Twist):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def joint_state_callback(self, msg: JointState):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = 1.0 / FREQUENCY
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_angles:
                vel = (pos - self.joint_angles[name]) / dt
                self.joint_velocities[name] = vel
                self.joint_angles[name] = pos

    def update(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # Check if we should switch tripods
        if self.linear_x != 0.0 or self.angular_z != 0.0:
            done = all(
                abs(self.target_angles[j] - self.joint_angles[j]) < 0.05
                for j in self.current_tripod
            )
            if done and now - self.phase_start_time > 1.0:  # wait 1s after reaching goal
                self.current_tripod, self.waiting_tripod = self.waiting_tripod, self.current_tripod
                step_direction = STEP_SIZE if self.linear_x >= 0 else -STEP_SIZE
                for j in self.current_tripod:
                    self.target_angles[j] = self.joint_angles[j] + step_direction
                # Set hold position for waiting tripod
                self.hold_position = {j: self.joint_angles[j] for j in self.waiting_tripod}
                self.phase_start_time = now
                self.get_logger().info(f"Switched tripod: {self.current_tripod}")

        # Compute PID commands
        commands = []
        for j in ALL_JOINTS:
            velocity = self.joint_velocities[j]
            if j in self.current_tripod:
                error = self.target_angles[j] - self.joint_angles[j]
                cmd = self.motion_pid.compute(error, velocity)
            else:
                # Soft hold
                hold_error = self.hold_position[j] - self.joint_angles[j]
                cmd = self.hold_pid.compute(hold_error, velocity)
            commands.append(cmd)

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
