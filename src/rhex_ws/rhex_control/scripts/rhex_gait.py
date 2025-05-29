#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

FREQUENCY = 100.0  # Hz
GAIT_PERIOD = 1.0  # seconds for full gait cycle
DUTY_FACTOR = 0.5  # percent of time in stance
dt = 1.0 / FREQUENCY

TRIPOD_A = ['front_left_leg_joint', 'centre_right_leg_joint', 'back_left_leg_joint']
TRIPOD_B = ['front_right_leg_joint', 'centre_left_leg_joint', 'back_right_leg_joint']
ALL_JOINTS = TRIPOD_A + TRIPOD_B

PHASES = {
    'front_left_leg_joint': 0.0,
    'centre_right_leg_joint': 0.5,
    'back_left_leg_joint': 0.0,
    'front_right_leg_joint': 0.5,
    'centre_left_leg_joint': 0.0,
    'back_right_leg_joint': 0.5,
}

class PIDController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd

    def compute(self, pos_error, vel_error):
        return self.kp * pos_error + self.kd * vel_error

class RHexTripodGaitController(Node):
    def __init__(self):
        super().__init__('rhex_tripod_gait_controller')

        self.publisher = self.create_publisher(Float64MultiArray, '/robot1/velocity_controller/commands', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/robot1/joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(dt, self.update)

        self.linear_x = 0.0
        self.angular_z = 0.0

        self.joint_angles = {j: 0.0 for j in ALL_JOINTS}
        self.joint_velocities = {j: 0.0 for j in ALL_JOINTS}
        self.pid = {j: PIDController(kp=5.0, kd=0.5) for j in ALL_JOINTS}

        self.get_logger().info("MiniRHex-style tripod gait controller started.")

    def cmd_vel_callback(self, msg: Twist):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def joint_state_callback(self, msg: JointState):
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name in self.joint_angles:
                self.joint_angles[name] = pos
                self.joint_velocities[name] = vel

    def get_gait_target(self, joint_name, time_sec):
        phase_offset = PHASES[joint_name]
        t = (time_sec + phase_offset * GAIT_PERIOD) % GAIT_PERIOD
        t_stance = DUTY_FACTOR * GAIT_PERIOD

        if t < t_stance:
            # Stance phase: slow backward sweep
            theta = -0.5 + (t / t_stance)
            velocity = 1.0 / t_stance
        else:
            # Swing phase: fast forward return
            t_swing = GAIT_PERIOD - t_stance
            theta = 0.5 - ((t - t_stance) / t_swing)
            velocity = -1.0 / t_swing

        # Scale based on linear velocity
        return theta * self.linear_x, velocity * self.linear_x

    def update(self):
        now = self.get_clock().now().nanoseconds / 1e9
        commands = []

        for j in ALL_JOINTS:
            theta_des, vel_des = self.get_gait_target(j, now)
            theta_act = self.joint_angles[j]
            vel_act = self.joint_velocities[j]
            pos_error = theta_des - theta_act
            vel_error = vel_des - vel_act
            cmd = self.pid[j].compute(pos_error, vel_error)
            commands.append(cmd)

        msg = Float64MultiArray()
        msg.data = commands
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RHexTripodGaitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
