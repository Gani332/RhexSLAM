#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time

TRIPOD_A = ['front_left_leg_joint', 'centre_right_leg_joint', 'back_left_leg_joint']
TRIPOD_B = ['front_right_leg_joint', 'centre_left_leg_joint', 'back_right_leg_joint']
ALL_JOINTS = TRIPOD_A + TRIPOD_B
STEP_AMOUNT = 5.80  # radians
KP = 4.0
KD = 1.0
EPSILON = 0.1  # Position threshold
VEL_THRESHOLD = 0.1  # Velocity threshold

class RHexSimpleStepper(Node):
    def __init__(self):
        super().__init__('rhex_simple_stepper')

        self.publisher = self.create_publisher(Float64MultiArray, '/robot1/velocity_controller/commands', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/robot1/joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(0.01, self.update)  # 100 Hz

        self.joint_angles = {j: 0.0 for j in ALL_JOINTS}
        self.joint_velocities = {j: 0.0 for j in ALL_JOINTS}
        self.target_angles = {j: 0.0 for j in ALL_JOINTS}
        self.leg_done = {j: False for j in ALL_JOINTS}

        self.current_tripod = TRIPOD_A
        self.waiting_tripod = TRIPOD_B

        self.initialized = False
        self.in_pause = False
        self.pause_start_time = None
        self.pause_duration = 2.0  # seconds
        self.stepping = False

        self.get_logger().info("RHex per-joint stepper with PD and pause started.")

    def joint_state_callback(self, msg: JointState):
        dt = 0.01
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_angles:
                prev_pos = self.joint_angles[name]
                self.joint_velocities[name] = (pos - prev_pos) / dt
                self.joint_angles[name] = pos

    def update(self):
        now = time.time()

        self.get_logger().info("------ Step Status ------")
        for j in self.current_tripod:
            pos = self.joint_angles[j]
            vel = self.joint_velocities[j]
            tgt = self.target_angles[j]
            pos_err = tgt - pos
            done = self.leg_done[j]
            self.get_logger().info(
                f"{j}: pos={pos:.2f}, tgt={tgt:.2f}, err={pos_err:.2f}, vel={vel:.2f}, done={done}"
            )


        if not self.initialized:
            if any(self.joint_angles.values()):
                # Set first targets
                for j in self.current_tripod:
                    self.target_angles[j] = self.joint_angles[j] + STEP_AMOUNT
                    self.leg_done[j] = False
                self.stepping = True
                self.initialized = True
                self.get_logger().info("Initialized and starting first tripod step.")
            return

        if self.in_pause:
            if now - self.pause_start_time >= self.pause_duration:
                self.in_pause = False
                # Set new targets for the next tripod
                for j in self.current_tripod:
                    self.target_angles[j] = self.joint_angles[j] + STEP_AMOUNT
                    self.leg_done[j] = False
                self.stepping = True
                self.get_logger().info(f"Resuming step for tripod: {self.current_tripod}")
            else:
                self.publish_velocity([0.0] * len(ALL_JOINTS))
                return

        if self.stepping:
            # Check per-leg completion
            for j in self.current_tripod:
                if not self.leg_done[j]:
                    pos_err = abs(self.target_angles[j] - self.joint_angles[j])
                    vel = abs(self.joint_velocities[j])
                    if pos_err < EPSILON and vel < VEL_THRESHOLD:
                        self.leg_done[j] = True
                        self.get_logger().info(f"{j} reached target.")

            if all(self.leg_done[j] for j in self.current_tripod):
                self.publish_velocity([0.0] * len(ALL_JOINTS))
                self.get_logger().info(f"Step complete for tripod: {self.current_tripod}. Pausing...")

                self.current_tripod, self.waiting_tripod = self.waiting_tripod, self.current_tripod
                self.in_pause = True
                self.pause_start_time = now
                self.stepping = False
                return

        # Apply PD control to current tripod
        self.publish_pd_velocity()

    def publish_pd_velocity(self):
        commands = []
        for j in ALL_JOINTS:
            if j in self.current_tripod:
                error = self.target_angles[j] - self.joint_angles[j]
                velocity = self.joint_velocities[j]
                cmd = KP * error - KD * velocity
            else:
                cmd = 0.0
            commands.append(cmd)

        msg = Float64MultiArray()
        msg.data = commands
        self.publisher.publish(msg)

    def publish_velocity(self, velocity_list):
        msg = Float64MultiArray()
        msg.data = velocity_list
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RHexSimpleStepper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
