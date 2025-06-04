#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
from geometry_msgs.msg import Twist

STEP_SIZE=5.40
FREQUENCY=100

class RHexSimpleStepper(Node):
    def __init__(self):
        super().__init__('rhex_simple_stepper')

        self.publisher = self.create_publisher(Float64MultiArray, '/robot1/velocity_controller/commands', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/robot1/joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(0.01, self.update)  # 100 Hz
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.moving=False
        self.step_direction=STEP_SIZE


    def cmd_vel_callback(self, msg: Twist):
        if abs(msg.linear.x) > 0.1:
            self.moving = True
            self.step_direction = STEP_SIZE if msg.linear.x > 0 else -STEP_SIZE
        else:
            self.moving = False


    def joint_state_callback(self, msg: JointState):
        dt = 1.0 / FREQUENCY
        self.get_logger().info("Received JointState:")
        for name, pos in zip(msg.name, msg.position):
            self.get_logger().info(f"  {name}: {pos:.2f}")
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_angles:
                vel = (pos - self.joint_angles[name]) / dt
                self.joint_velocities[name] = vel
                self.joint_angles[name] = pos
    
    def update(self):
        return 0




def main(args=None):
    rclpy.init(args=args)
    node = RHexSimpleStepper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
