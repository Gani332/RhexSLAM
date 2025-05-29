#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

FREQUENCY = 100.0
STEP_SIZE = 1.5      # radians to move each step
STEP_THRESHOLD = 5.50  # radians to travel before switching tripod

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

        self.in_grounded_pause = False
        self.pause_start_time = 0.0
        self.pause_duration = 0.4  # seconds (adjustable)

        # Motion command state
        self.linear_x = 0.0
        self.angular_z = 0.0

        # Tripod state
        self.current_tripod = TRIPOD_A
        self.waiting_tripod = TRIPOD_B

        # Encoder/joint state tracking
        self.joint_angles = {j: 0.0 for j in ALL_JOINTS}
        self.joint_velocities = {j: 0.0 for j in ALL_JOINTS}
        self.target_angles = {j: 0.0 for j in ALL_JOINTS}
        self.hold_position = {j: 0.0 for j in ALL_JOINTS}
        self.step_start_position = {j: 0.0 for j in ALL_JOINTS}

        # PID controller per joint
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
        now = self.get_clock().now().nanoseconds / 1e9

        # Exit pause if time has passed
        if self.in_grounded_pause:
            if now - self.pause_start_time >= self.pause_duration:
                self.in_grounded_pause = False
                step_direction = STEP_SIZE if self.linear_x >= 0 else -STEP_SIZE
                for j in self.current_tripod:
                    self.target_angles[j] = self.joint_angles[j] + step_direction
                    self.step_start_position[j] = self.joint_angles[j]
                for j in self.waiting_tripod:
                    self.hold_position[j] = self.joint_angles[j]
                self.get_logger().info(f"Exited grounded pause. Starting: {self.current_tripod}")
            else:
                # Still pausing — apply damping to all joints below
                pass

        # First step trigger
        if self.linear_x != 0.0 and all(v == 0.0 for v in self.target_angles.values()):
            step_direction = STEP_SIZE if self.linear_x >= 0 else -STEP_SIZE
            for j in self.current_tripod:
                self.target_angles[j] = self.joint_angles[j] + step_direction
                self.step_start_position[j] = self.joint_angles[j]
            for j in self.waiting_tripod:
                self.hold_position[j] = self.joint_angles[j]
            self.get_logger().info("Initialized first step.")

        # Check for phase switch if not pausing
        if not self.in_grounded_pause and (
            all(
                abs(self.joint_angles[j] - self.step_start_position.get(j, 0.0)) >= STEP_THRESHOLD
                for j in self.current_tripod
                if "centre" in j
            ) and (self.linear_x != 0.0 or self.angular_z != 0.0)
        ):
            # Swap tripods and enter pause
            self.current_tripod, self.waiting_tripod = self.waiting_tripod, self.current_tripod
            self.pause_start_time = now
            self.in_grounded_pause = True
            self.get_logger().info("Entered grounded pause.")

        # Compute PID commands
        commands = []
        for j in ALL_JOINTS:
            if self.in_grounded_pause or j in self.waiting_tripod:
                # Passive damping for stability
                velocity = self.joint_velocities[j]
                damping_gain = 1.5
                stiffness_gain = 0.1
                hold_pos = self.hold_position.get(j, self.joint_angles[j])
                error = hold_pos - self.joint_angles[j]
                cmd = stiffness_gain * error - damping_gain * velocity
                commands.append(cmd)
            else:
                # Active step with PID
                error = self.target_angles[j] - self.joint_angles[j]
                velocity = self.joint_velocities[j]
                cmd = self.pid[j].compute(error, velocity)
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
