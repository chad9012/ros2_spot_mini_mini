#!/usr/bin/env python3
"""
Sends a standing pose to joint_trajectory_controller.
Run once after controllers activate to prevent ragdoll collapse.
This is a temporary hold until full teleop + policy migration is complete.
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# Joint order must match what /joint_states publishes
JOINT_NAMES = [
    'motor_front_left_hip',
    'motor_front_left_upper_leg',
    'motor_front_left_lower_leg',
    'motor_front_right_hip',
    'motor_front_right_upper_leg',
    'motor_front_right_lower_leg',
    'motor_back_left_hip',
    'motor_back_left_upper_leg',
    'motor_back_left_lower_leg',
    'motor_back_right_hip',
    'motor_back_right_upper_leg',
    'motor_back_right_lower_leg',
]

# Standing pose
# hip        = 0.0   straight, no sideways lean
# upper_leg  = 0.785 45 degrees forward and down
# lower_leg  = -1.57 90 degrees bent back under body
STANDING_POSITIONS = [
    0.0,    # front_left_hip
    0.785,  # front_left_upper_leg
   -1.57,   # front_left_lower_leg
    0.0,    # front_right_hip
    0.785,  # front_right_upper_leg
   -1.57,   # front_right_lower_leg
    0.0,    # back_left_hip
    0.785,  # back_left_upper_leg
   -1.57,   # back_left_lower_leg
    0.0,    # back_right_hip
    0.785,  # back_right_upper_leg
   -1.57,   # back_right_lower_leg
]


class HoldPosition(Node):
    def __init__(self):
        super().__init__('hold_position')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/spot/joint_trajectory_controller/joint_trajectory',
            10
        )
        # Send command after 2 seconds
        self.timer = self.create_timer(2.0, self.send_command)
        self.get_logger().info('Waiting 2s then sending standing pose...')

    def send_command(self):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = STANDING_POSITIONS
        point.velocities = [0.0] * 12
        # Move to standing pose over 2 seconds
        point.time_from_start = Duration(sec=2, nanosec=0)

        msg.points = [point]
        self.publisher.publish(msg)
        self.get_logger().info('Standing pose sent!')
        self.timer.cancel()


def main():
    rclpy.init()
    node = HoldPosition()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()