#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
from geometry_msgs.msg import Twist

STEP_SIZE=6.28
FREQUENCY=100
KP = 1.0
KD = 2.0

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


class RHexSimpleStepper(Node):
    def __init__(self):
        super().__init__('rhex_simple_stepper')

        self.publisher = self.create_publisher(Float64MultiArray, '/robot1/velocity_controller/commands', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/robot1/joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(0.01, self.update)  # 100 Hz
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.joint_angles = {j: 0.0 for j in ALL_JOINTS}
        self.joint_velocities = {j: 0.0 for j in ALL_JOINTS}
        self.step_start_angles = {j: 0.0 for j in ALL_JOINTS}
        self.step_index = {j: 0 for j in ALL_JOINTS}

        self.current_tripod = TRIPOD_A
        self.waiting_tripod = TRIPOD_B

        self.moving=False
        self.initialized=False
        self.step_direction=STEP_SIZE
        self.stepping=False


    def cmd_vel_callback(self, msg: Twist):
        if abs(msg.linear.x) > 0.1:
            self.moving = True
            self.step_direction = STEP_SIZE if msg.linear.x > 0 else -STEP_SIZE
        else:
            self.moving = False
            self.stepping = False
            self.publish_stop()


    def joint_state_callback(self, msg: JointState):
        dt = 1.0 / FREQUENCY
        # self.get_logger().info("Received JointState:")
        # for name, pos in zip(msg.name, msg.position):
        #     self.get_logger().info(f"  {name}: {pos:.2f}")
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_angles:
                vel = (pos - self.joint_angles[name]) / dt
                self.joint_velocities[name] = vel
                self.joint_angles[name] = pos
    
    def publish_stop(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * len(ALL_JOINTS)
        self.publisher.publish(msg)


    def publish_pd_command(self, tripod):
        commands = []
        for j in ALL_JOINTS:
            if j in tripod:
                target = self.step_index[j] * self.step_direction
                error = target - self.joint_angles[j]
                vel = self.joint_velocities[j]
                cmd = KP * error - KD * vel
            else:
                cmd = 0.0
            commands.append(cmd)

        msg = Float64MultiArray()
        msg.data = commands
        self.publisher.publish(msg)




    def update(self):
        if not self.moving:
            return

        if not self.stepping:
            for leg in self.current_tripod:
                self.step_index[leg] += 1
            self.stepping = True
            self.get_logger().info(f"Starting step for {self.current_tripod}")
            return


        # Check progress of the step
        done = all(
            abs(self.joint_angles[leg]) >= abs(self.step_direction)
            for leg in self.current_tripod
        )

        if done:
            self.publish_stop()
            self.current_tripod, self.waiting_tripod = self.waiting_tripod, self.current_tripod
            self.stepping = False
            self.get_logger().info(f"Step complete. Switching tripod.")
        else:
            self.publish_pd_command(self.current_tripod)





def main(args=None):
    rclpy.init(args=args)
    node = RHexSimpleStepper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
