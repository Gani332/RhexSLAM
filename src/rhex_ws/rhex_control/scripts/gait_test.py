#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time

TRIPOD_A = ['front_left_leg_joint', 'centre_right_leg_joint', 'back_left_leg_joint']
TRIPOD_B = ['front_right_leg_joint', 'centre_left_leg_joint', 'back_right_leg_joint']
ALL_JOINTS = TRIPOD_A + TRIPOD_B
STEP_AMOUNT = 5.40  # 2π radians

KP = 2.0
KD = 1.0

class RHexSimpleStepper(Node):
    def __init__(self):
        super().__init__('rhex_simple_stepper')

        self.publisher = self.create_publisher(Float64MultiArray, '/robot1/velocity_controller/commands', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/robot1/joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(0.01, self.update)  # 100 Hz

        self.joint_angles = {j: 0.0 for j in ALL_JOINTS}
        self.joint_velocities = {j: 0.0 for j in ALL_JOINTS}
        self.last_joint_angles = {j: 0.0 for j in ALL_JOINTS}

        self.current_tripod = TRIPOD_A
        self.waiting_tripod = TRIPOD_B
        self.center_leg = self.get_center_leg(self.current_tripod)
        self.step_start_angles = {j: 0.0 for j in ALL_JOINTS}
        self.stepping = False

        self.initialized = False
        self.pause_start_time = None
        self.in_pause = False
        self.pause_duration = 2.0  # seconds

        self.get_logger().info("Simple RHex tripod stepper with PD and pause started.")

    def get_center_leg(self, tripod):
        return next(j for j in tripod if 'centre' in j)

    def joint_state_callback(self, msg: JointState):
        dt = 0.01  # Fixed timestep for now
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_angles:
                prev_pos = self.joint_angles[name]
                vel = (pos - prev_pos) / dt
                self.joint_angles[name] = pos
                self.joint_velocities[name] = vel

        # DEBUG: Print current joint angles
        self.get_logger().info("Joint Angles:")
        for j in ALL_JOINTS:
            self.get_logger().info(f"  {j}: {self.joint_angles[j]:.2f}")

    def update(self):
        now = time.time()

        if not self.initialized:
            if any(self.joint_angles.values()):
                for leg in self.current_tripod:
                    self.step_start_angles[leg] = self.joint_angles[leg]
                self.stepping = True
                self.initialized = True
                self.get_logger().info(f"Initialized stepping with {self.center_leg}")
            return

        if self.in_pause:
            if now - self.pause_start_time >= self.pause_duration:
                self.in_pause = False
                for leg in self.current_tripod:
                    self.step_start_angles[leg] = self.joint_angles[leg]
                self.get_logger().info(f"Resuming stepping for {self.center_leg}")
            else:
                self.publish_velocity([0.0] * len(ALL_JOINTS))
                return

        if self.stepping:
            # DEBUG: Check how far each leg in tripod has moved
            for leg in self.current_tripod:
                moved = abs(self.joint_angles[leg] - self.step_start_angles[leg])
                self.get_logger().info(f"{leg} moved: {moved:.2f} / {STEP_AMOUNT:.2f}")

            all_reached = all(
                abs(self.joint_angles[leg] - self.step_start_angles[leg]) >= STEP_AMOUNT
                for leg in self.current_tripod
            )
            if all_reached:
                self.publish_velocity([0.0] * len(ALL_JOINTS))
                self.get_logger().info(f"Step complete for {self.center_leg}, pausing.")

                # Switch tripod
                self.current_tripod, self.waiting_tripod = self.waiting_tripod, self.current_tripod
                self.center_leg = self.get_center_leg(self.current_tripod)

                self.in_pause = True
                self.pause_start_time = now
                return
            else:
                self.publish_pd_velocity(self.current_tripod)

    def publish_pd_velocity(self, tripod):
        commands = []
        for j in ALL_JOINTS:
            if j in tripod:
                target_angle = self.step_start_angles[j] + STEP_AMOUNT
                error = target_angle - self.joint_angles[j]
                velocity = self.joint_velocities[j]
                cmd = KP * error - KD * velocity
            else:
                cmd = 0.0
            commands.append(cmd)

        # DEBUG: Print motor commands
        self.get_logger().info("Motor Commands:")
        for j, cmd in zip(ALL_JOINTS, commands):
            self.get_logger().info(f"  {j}: {cmd:.2f}")

        msg = Float64MultiArray()
        msg.data = commands
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RHexSimpleStepper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
