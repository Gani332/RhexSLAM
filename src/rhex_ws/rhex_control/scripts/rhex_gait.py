#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

FREQUENCY = 100         # Hz
GAIT_CYCLE = 0.5        # seconds for full tripod cycle
SWING_SPEED = 1       # rad/s during swing phase
SUPPORT_SPEED = 0.0     # no movement in support

ALL_JOINTS = [
    'front_left_leg_joint',    # 0
    'centre_left_leg_joint',   # 1
    'back_left_leg_joint',     # 2
    'front_right_leg_joint',   # 3
    'centre_right_leg_joint',  # 4
    'back_right_leg_joint'     # 5
]

TRIPOD_A = [0, 4, 2]  # FL, CR, BL
TRIPOD_B = [3, 1, 5]  # FR, CL, BR


class RHexOpenLoopWalker(Node):
    def __init__(self):
        super().__init__('rhex_open_loop_walker')

        self.publisher = self.create_publisher(Float64MultiArray, '/robot1/velocity_controller/commands', 10)
        self.timer = self.create_timer(1.0 / FREQUENCY, self.update)

        self.start_time = time.time()

    def update(self):
        t = time.time() - self.start_time
        cycle_time = t % GAIT_CYCLE
        half_cycle = GAIT_CYCLE / 2

        # Decide which tripod is swinging
        if cycle_time < half_cycle:
            swing = TRIPOD_A
        else:
            swing = TRIPOD_B

        # Set leg velocities
        velocities = []
        for i in range(len(ALL_JOINTS)):
            if i in swing:
                velocities.append(SWING_SPEED)
            else:
                velocities.append(SUPPORT_SPEED)

        msg = Float64MultiArray()
        msg.data = velocities
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RHexOpenLoopWalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
