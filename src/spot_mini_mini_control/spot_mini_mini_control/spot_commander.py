#!/usr/bin/env python3
"""
spot_commander.py — ROS2 migration of spot_pybullet_interface

WHAT CHANGED FROM ROS1:
  - rospy → rclpy
  - spotBezierEnv (PyBullet) completely removed
  - joint angles now published directly to /spot/joint_trajectory_controller
  - contacts from PyBullet replaced with assumed [1,1,1,1]
  - dt from PyBullet timestep replaced with ROS2 timer period
  - rate 600Hz → timer callback at same frequency

WHAT STAYED THE SAME:
  - BezierGait.GenerateTrajectory() — identical
  - SpotModel.IK() — identical
  - move() logic — identical
  - All scaling constants — identical
  - MiniCmd/JoyButtons message handling — identical
"""

import copy
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from spot_mini_mini_interfaces.msg import MiniCmd, JoyButtons

# Pure math imports — no ROS, no PyBullet
from spot_mini_mini_control.spotmicro.Kinematics.SpotKinematics import SpotModel
from spot_mini_mini_control.spotmicro.GaitGenerator.Bezier import BezierGait

# ── Controller Scaling Constants ─────────────────────────────────────────────
# Identical to original spot_pybullet_interface.py
STEPLENGTH_SCALE = 0.06   # right joystick up/down
Z_SCALE_CTRL     = 0.12   # left joystick up/down (height)
RPY_SCALE        = 0.6    # body orientation in viewing mode
SV_SCALE         = 0.1    # step velocity scale (bumpers)
CHPD_SCALE       = 0.0005 # clearance/penetration depth (arrow pad)
YAW_SCALE        = 1.5    # yaw rate scale

# ── Joint Name Mapping ────────────────────────────────────────────────────────
# BezierGait + SpotModel outputs joint angles in order: FL, FR, BL, BR
# Each leg: [hip, upper_leg, lower_leg]
# Must match URDF joint names exactly

JOINT_NAMES = [
    # FL leg (index 0 in joint_angles array)
    'motor_front_left_hip',
    'motor_front_left_upper_leg',
    'motor_front_left_lower_leg',
    # FR leg (index 1)
    'motor_front_right_hip',
    'motor_front_right_upper_leg',
    'motor_front_right_lower_leg',
    # BL leg (index 2)
    'motor_back_left_hip',
    'motor_back_left_upper_leg',
    'motor_back_left_lower_leg',
    # BR leg (index 3)
    'motor_back_right_hip',
    'motor_back_right_upper_leg',
    'motor_back_right_lower_leg',
]

# Control frequency — same as original ROS1 code
CONTROL_HZ = 50.0


class SpotCommander(Node):
    def __init__(self):
        super().__init__('spot_commander')

        self.get_logger().info('Initializing Spot Commander...')

        # ── Internal State ────────────────────────────────────────────
        self.mini_cmd = MiniCmd()
        self.mini_cmd.x_velocity = 0.0
        self.mini_cmd.y_velocity = 0.0
        self.mini_cmd.rate       = 0.0
        self.mini_cmd.roll       = 0.0
        self.mini_cmd.pitch      = 0.0
        self.mini_cmd.yaw        = 0.0
        self.mini_cmd.z          = 0.0
        self.mini_cmd.motion     = "Stop"
        self.mini_cmd.movement   = "Stepping"

        self.jb = JoyButtons()
        self.jb.updown     = 0
        self.jb.leftright  = 0
        self.jb.left_bump  = False
        self.jb.right_bump = False

        # Gait parameters — same defaults as original
        self.BaseStepVelocity    = 0.1
        self.StepVelocity        = self.BaseStepVelocity
        self.BaseSwingPeriod     = 0.2
        self.SwingPeriod         = self.BaseSwingPeriod
        self.BaseClearanceHeight = 0.04
        self.BasePenetrationDepth= 0.005
        self.ClearanceHeight     = self.BaseClearanceHeight
        self.PenetrationDepth    = self.BasePenetrationDepth

        # ── Load Pure Math Components ─────────────────────────────────
        # ROS1: self.env = spotBezierEnv(...) then extracted dt and WorldToFoot
        # ROS2: directly instantiate SpotModel and BezierGait
        # dt = 1/600Hz = 0.00167s (same as PyBullet was using internally)
        self.dt = 1.0 / CONTROL_HZ

        self.spot = SpotModel()

        # Initial foot positions from SpotModel
        # ROS1: self.T_bf0 = self.spot.WorldToFoot after env.reset()
        # ROS2: take directly from SpotModel constructor
        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf  = copy.deepcopy(self.T_bf0)

        self.bzg = BezierGait(dt=self.dt)

        # ── ROS2 Publishers ───────────────────────────────────────────
        # ROS1: env.pass_joint_angles() sent to PyBullet
        # ROS2: publish directly to joint_trajectory_controller
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/spot/joint_trajectory_controller/joint_trajectory',
            10
        )

        # ── ROS2 Subscribers ──────────────────────────────────────────
        # ROS1: rospy.Subscriber('mini_cmd', MiniCmd, self.mini_cmd_cb)
        # ROS2: create_subscription with lambda
        self.mini_cmd_sub = self.create_subscription(
            MiniCmd,
            'mini_cmd',
            self.mini_cmd_cb,
            10
        )

        self.jb_sub = self.create_subscription(
            JoyButtons,
            'joybuttons',
            self.jb_cb,
            10
        )

        # ── Control Timer ─────────────────────────────────────────────
        # ROS1: while not rospy.is_shutdown(): move(); rate.sleep()
        # ROS2: timer callback at same 600Hz frequency
        self.timer = self.create_timer(self.dt, self.move)

        self.get_logger().info('Spot Commander ready at %.0f Hz!' % CONTROL_HZ)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def mini_cmd_cb(self, mini_cmd):
        """
        ROS1: rospy.logdebug inside try/except ROSInterruptException
        ROS2: simple assignment, no try/except needed
        """
        self.mini_cmd = mini_cmd
        self.get_logger().debug(
            'MiniCmd: motion=%s movement=%s vx=%.2f' %
            (mini_cmd.motion, mini_cmd.movement, mini_cmd.x_velocity)
        )

    def jb_cb(self, jb):
        self.jb = jb

    # ── Main Control Loop ─────────────────────────────────────────────────────

    def move(self):
        """
        Core control logic — nearly identical to original move()
        Only change: env.pass_joint_angles() → publish_joint_trajectory()
        and contacts = self.state[-4:] → contacts = [1,1,1,1]
        """

        if self.mini_cmd.motion != "Stop":
            self.StepVelocity = self.BaseStepVelocity
            self.SwingPeriod = np.clip(
                self.BaseSwingPeriod +
                (-self.mini_cmd.faster +self.mini_cmd.slower) * SV_SCALE,
                0.1, 0.3
            )

            if self.mini_cmd.movement == "Stepping":
                # Stepping mode — velocity commands
                StepLength = self.mini_cmd.x_velocity + abs(
                    self.mini_cmd.y_velocity * 0.66)
                StepLength = np.clip(StepLength, -1.0, 1.0)
                StepLength *= STEPLENGTH_SCALE
                LateralFraction = self.mini_cmd.y_velocity * np.pi / 2
                YawRate = self.mini_cmd.rate * YAW_SCALE
                pos = np.array([0.0, 0.0, self.mini_cmd.z * Z_SCALE_CTRL])
                orn = np.array([0.0, 0.0, 0.0])

            else:
                # Viewing mode — body pose commands
                StepLength    = 0.0
                LateralFraction = 0.0
                YawRate       = 0.0
                self.ClearanceHeight  = self.BaseClearanceHeight
                self.PenetrationDepth = self.BasePenetrationDepth
                self.StepVelocity     = self.BaseStepVelocity
                pos = np.array([0.0, 0.0, self.mini_cmd.z * Z_SCALE_CTRL])
                orn = np.array([
                    self.mini_cmd.roll  * RPY_SCALE,
                    self.mini_cmd.pitch * RPY_SCALE,
                    self.mini_cmd.yaw   * RPY_SCALE
                ])

        else:
            # Stop — reset everything
            StepLength      = 0.0
            LateralFraction = 0.0
            YawRate         = 0.0
            self.ClearanceHeight  = self.BaseClearanceHeight
            self.PenetrationDepth = self.BasePenetrationDepth
            self.StepVelocity     = self.BaseStepVelocity
            self.SwingPeriod      = self.BaseSwingPeriod
            pos = np.array([0.0, 0.0, 0.0])
            orn = np.array([0.0, 0.0, 0.0])

        # Arrow pad — adjust clearance and penetration depth
        self.ClearanceHeight  += self.jb.updown    * CHPD_SCALE
        self.PenetrationDepth += self.jb.leftright * CHPD_SCALE

        # Manual reset via bumpers
        if self.jb.left_bump or self.jb.right_bump:
            self.get_logger().info('MANUAL RESET triggered')
            self.ClearanceHeight  = self.BaseClearanceHeight
            self.PenetrationDepth = self.BasePenetrationDepth
            self.StepVelocity     = self.BaseStepVelocity
            self.SwingPeriod      = self.BaseSwingPeriod
            self.T_bf = copy.deepcopy(self.T_bf0)

        # ── Contact Sensors ───────────────────────────────────────────
        # ROS1: contacts = self.state[-4:] from PyBullet observation
        # ROS2: assume all feet in contact for now
        # TODO: get real contact data from Gazebo contact sensors
        contacts = np.array([1, 1, 1, 1])

        # ── Update Gait ───────────────────────────────────────────────
        # Identical to original — pure math, no changes needed
        self.bzg.Tswing = self.SwingPeriod

        self.T_bf = self.bzg.GenerateTrajectory(
            StepLength,
            LateralFraction,
            YawRate,
            self.StepVelocity,
            self.T_bf0,
            self.T_bf,
            self.ClearanceHeight,
            self.PenetrationDepth,
            contacts,
            self.dt          # ROS1 used rospy.get_time() delta, we use fixed dt
        )

        # ── Inverse Kinematics ────────────────────────────────────────
        # Identical to original — pure math
        joint_angles = self.spot.IK(orn, pos, self.T_bf)

        # ── Publish to Gazebo / Real Robot ────────────────────────────
        # ROS1: self.env.pass_joint_angles(joint_angles.reshape(-1))
        # ROS2: publish JointTrajectory message to controller
        self.publish_joint_trajectory(joint_angles)

    def publish_joint_trajectory(self, joint_angles):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = joint_angles.flatten().tolist()
        # NO velocities — let controller interpolate smoothly
        # NO time_from_start — zero means "execute immediately"

        msg.points = [point]
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpotCommander()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
