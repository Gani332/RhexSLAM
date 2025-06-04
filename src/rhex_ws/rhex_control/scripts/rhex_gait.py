#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import time
import math

# Control parameters
KP = 0.1
KD = 1.0
FREQUENCY = 100

# Gait parameters
GAIT_CYCLE_PERIOD = 1.0  # seconds
SWING_START_ANGLE = -0.5
SWING_END_ANGLE = 0.5
SUPPORT_ANGLE = 0.0

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
    return math.atan2(math.sin(angle), math.cos(angle))

def swing_trajectory(phase):
    """Smooth swing trajectory from back to front using cosine easing."""
    return SWING_START_ANGLE + (SWING_END_ANGLE - SWING_START_ANGLE) * 0.5 * (1 - math.cos(math.pi * phase))

def get_desired_angles(time_elapsed, step_indices):
    desired_angles = [0.0] * len(ALL_JOINTS)
    t = time_elapsed % GAIT_CYCLE_PERIOD
    half_T = GAIT_CYCLE_PERIOD / 2

    if t < half_T:
        phase = t / half_T
        swing_tripod = TRIPOD_A
        stance_tripod = TRIPOD_B
    else:
        phase = (t - half_T) / half_T
        swing_tripod = TRIPOD_B
        stance_tripod = TRIPOD_A

    for i, joint in enumerate(ALL_JOINTS):
        step = step_indices[joint]
        if joint in swing_tripod:
            # Compute next target step
            start = step * (SWING_END_ANGLE - SWING_START_ANGLE)
            end = (step + 1) * (SWING_END_ANGLE - SWING_START_ANGLE)
            angle = start + (end - start) * 0.5 * (1 - math.cos(math.pi * phase))
            desired_angles[i] = angle
        else:
            # Hold last reached support position
            desired_angles[i] = step * (SWING_END_ANGLE - SWING_START_ANGLE)

    return desired_angles



class RHexSmoothGait(Node):
    def __init__(self):
        super().__init__('rhex_smooth_gait')

        self.publisher = self.create_publisher(Float64MultiArray, '/robot1/velocity_controller/commands', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/robot1/joint_states', self.joint_state_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(1.0 / FREQUENCY, self.update)

        self.step_indices = {j: 0 for j in ALL_JOINTS}
        self.last_phase = 0.0

        self.joint_angles = {j: 0.0 for j in ALL_JOINTS}
        self.joint_velocities = {j: 0.0 for j in ALL_JOINTS}
        self.time_elapsed = 0.0
        self.moving = False

    def cmd_vel_callback(self, msg: Twist):
        if abs(msg.linear.x) > 0.1:
            self.moving = True
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

    def update(self):
        if not self.moving:
            return

        self.time_elapsed += 1.0 / FREQUENCY
        t = self.time_elapsed % GAIT_CYCLE_PERIOD
        half_T = GAIT_CYCLE_PERIOD / 2

        # Phase: 0 → 1 for each swing
        if t < half_T:
            phase = t / half_T
            active_tripod = TRIPOD_A
        else:
            phase = (t - half_T) / half_T
            active_tripod = TRIPOD_B

        # Detect phase wraparound to increment step index
        if self.last_phase > phase:
            for leg in active_tripod:
                self.step_indices[leg] += 1
        self.last_phase = phase

        desired_angles = get_desired_angles(self.time_elapsed, self.step_indices)

        commands = []
        for i, joint in enumerate(ALL_JOINTS):
            target = desired_angles[i]
            current = self.joint_angles[joint]
            vel = self.joint_velocities[joint]
            error = normalize_angle(target - current)
            cmd = KP * error - KD * vel
            commands.append(cmd)

        msg = Float64MultiArray()
        msg.data = commands
        self.publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = RHexSmoothGait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
