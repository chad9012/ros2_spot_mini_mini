#!/usr/bin/env python3
"""
spot_commander.py

ROS2 node that controls Spot Mini Mini using pretrained ARS policies.
Subscribes to /mini_cmd, /joybuttons, /spot/imu, /spot/contact
Publishes /spot/agent, /spot/joints

Converted from ROS1 to ROS2.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import copy
import os
import sys

from spot_mini_mini_interfaces.msg import MiniCmd, JoyButtons, IMUdata, ContactData, AgentData, JointAngles

# SpotMicro library imports (from spot_mini_mini_pybullet package)
from spot_mini_mini_pybullet.spotmicro.Kinematics.SpotKinematics import SpotModel
from spot_mini_mini_pybullet.spotmicro.GaitGenerator.Bezier import BezierGait

# ARS Agent imports (from spot_bullet)
from spot_mini_mini_pybullet.spot_bullet.src.ars_lib.ars import Normalizer, Policy


# Controller Params (with defaults)
STEPLENGTH_SCALE = 0.06
Z_SCALE_CTRL = 0.12
RPY_SCALE = 0.6
SV_SCALE = 0.1
CHPD_SCALE = 0.0005
YAW_SCALE = 1.5

# AGENT PARAMS (with defaults)
CD_SCALE = 0.01
SLV_SCALE = 0.1
RESIDUALS_SCALE = 0.01
Z_SCALE = 0.01
alpha = 0.5
actions_to_filter = -1


class SpotCommander(Node):
    def __init__(self):
        super().__init__('spot_agent_commander')

        # Declare all parameters with defaults
        self.declare_parameter('Agent', True)
        self.declare_parameter('contacts', False)
        self.declare_parameter('agent_num', 0)
        self.declare_parameter('STEPLENGTH_SCALE', 0.06)
        self.declare_parameter('Z_SCALE_CTRL', 0.12)
        self.declare_parameter('RPY_SCALE', 0.6)
        self.declare_parameter('SV_SCALE', 0.1)
        self.declare_parameter('CHPD_SCALE', 0.0005)
        self.declare_parameter('YAW_SCALE', 1.5)
        self.declare_parameter('CD_SCALE', 0.01)
        self.declare_parameter('SLV_SCALE', 0.1)
        self.declare_parameter('RESIDUALS_SCALE', 0.01)
        self.declare_parameter('Z_SCALE', 0.01)
        self.declare_parameter('alpha', 0.5)
        self.declare_parameter('actions_to_filter', -1)
        self.declare_parameter('BaseStepVelocity', 0.1)
        self.declare_parameter('Tswing', 0.2)
        self.declare_parameter('SwingPeriod_LIMITS', [0.1, 0.3])
        self.declare_parameter('BaseClearanceHeight', 0.04)
        self.declare_parameter('BasePenetrationDepth', 0.005)
        self.declare_parameter('ClearanceHeight_LIMITS', [0.02, 0.08])
        self.declare_parameter('PenetrationDepth_LIMITS', [-0.01, 0.01])
        self.declare_parameter('shoulder_length', 0.055)
        self.declare_parameter('elbow_length', 0.1075)
        self.declare_parameter('wrist_length', 0.035)
        self.declare_parameter('hip_x', 0.192)
        self.declare_parameter('hip_y', 0.085)
        self.declare_parameter('foot_x', 0.192)
        self.declare_parameter('foot_y', 0.085)
        self.declare_parameter('height', 0.17)
        self.declare_parameter('com_offset', 0.0)
        self.declare_parameter('dt', 0.01)

        # Get parameters
        self.Agent = self.get_parameter('Agent').value
        self.agent_num = self.get_parameter('agent_num').value
        self.enable_contact = self.get_parameter('contacts').value

        self.STEPLENGTH_SCALE = self.get_parameter('STEPLENGTH_SCALE').value
        self.Z_SCALE_CTRL = self.get_parameter('Z_SCALE_CTRL').value
        self.RPY_SCALE = self.get_parameter('RPY_SCALE').value
        self.SV_SCALE = self.get_parameter('SV_SCALE').value
        self.CHPD_SCALE = self.get_parameter('CHPD_SCALE').value
        self.YAW_SCALE = self.get_parameter('YAW_SCALE').value

        self.CD_SCALE = self.get_parameter('CD_SCALE').value
        self.SLV_SCALE = self.get_parameter('SLV_SCALE').value
        self.RESIDUALS_SCALE = self.get_parameter('RESIDUALS_SCALE').value
        self.Z_SCALE = self.get_parameter('Z_SCALE').value
        self.alpha = self.get_parameter('alpha').value
        self.actions_to_filter = self.get_parameter('actions_to_filter').value

        self.BaseStepVelocity = self.get_parameter('BaseStepVelocity').value
        self.BaseSwingPeriod = self.get_parameter('Tswing').value
        self.SwingPeriod_LIMITS = self.get_parameter('SwingPeriod_LIMITS').value
        self.BaseClearanceHeight = self.get_parameter('BaseClearanceHeight').value
        self.BasePenetrationDepth = self.get_parameter('BasePenetrationDepth').value
        self.ClearanceHeight_LIMITS = self.get_parameter('ClearanceHeight_LIMITS').value
        self.PenetrationDepth_LIMITS = self.get_parameter('PenetrationDepth_LIMITS').value

        # Initialize state
        self.movetypes = ["Stop"]
        self.mini_cmd = MiniCmd()
        self.jb = JoyButtons()
        self.mini_cmd.x_velocity = 0.0
        self.mini_cmd.y_velocity = 0.0
        self.mini_cmd.rate = 0.0
        self.mini_cmd.roll = 0.0
        self.mini_cmd.pitch = 0.0
        self.mini_cmd.yaw = 0.0
        self.mini_cmd.z = 0.0
        self.mini_cmd.motion = "Stop"
        self.mini_cmd.movement = "Stepping"

        self.StepVelocity = copy.deepcopy(self.BaseStepVelocity)
        self.SwingPeriod = copy.deepcopy(self.BaseSwingPeriod)
        self.ClearanceHeight = copy.deepcopy(self.BaseClearanceHeight)
        self.PenetrationDepth = copy.deepcopy(self.BasePenetrationDepth)

        # Time
        self.time = self.get_clock().now()

        # Contacts: FL, FR, BL, BR
        self.contacts = [0, 0, 0, 0]

        # IMU: R, P, Ax, Ay, Az, Gx, Gy, Gz
        self.imu = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Initialize Spot model
        self.spot = SpotModel(
            shoulder_length=self.get_parameter('shoulder_length').value,
            elbow_length=self.get_parameter('elbow_length').value,
            wrist_length=self.get_parameter('wrist_length').value,
            hip_x=self.get_parameter('hip_x').value,
            hip_y=self.get_parameter('hip_y').value,
            foot_x=self.get_parameter('foot_x').value,
            foot_y=self.get_parameter('foot_y').value,
            height=self.get_parameter('height').value,
            com_offset=self.get_parameter('com_offset').value)

        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf = copy.deepcopy(self.T_bf0)

        # Initialize Bezier Gait
        self.bzg = BezierGait(
            dt=self.get_parameter('dt').value,
            Tswing=self.get_parameter('Tswing').value)

        # Load all policies
        self.policies = {}
        self.normalizer = None
        if self.Agent:
            self.load_policies()

        # Current active policy
        self.current_policy_name = "Stop"
        self.policy = None
        self.state_dim = 12
        self.action_dim = 14

        # Subscribers
        self.sub_cmd = self.create_subscription(
            MiniCmd, 'mini_cmd', self.cmd_cb, 1)
        self.sub_jb = self.create_subscription(
            JoyButtons, 'joybuttons', self.jb_cb, 1)
        self.sub_imu = self.create_subscription(
            IMUdata, 'spot/imu', self.imu_cb, 1)
        self.sub_cnt = self.create_subscription(
            ContactData, 'spot/contact', self.cnt_cb, 1)

        # Publishers
        self.ag_pub = self.create_publisher(AgentData, 'spot/agent', 1)
        self.ja_pub = self.create_publisher(JointAngles, 'spot/joints', 1)

        # Timer for control loop at 600 Hz
        self.timer = self.create_timer(1.0 / 600.0, self.move)

        self.get_logger().info("SpotAgentCommander READY TO GO!")

    def load_policies(self):
        """Load all pretrained policies from the policies/ folder."""
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('spot_mini_mini_control')
        policies_dir = os.path.join(pkg_share, 'policies')

        self.get_logger().info("POLICIES DIR: {}".format(policies_dir))

        if not os.path.exists(policies_dir):
            self.get_logger().warn("Policies directory not found: {}".format(policies_dir))
            return

        # Map motion names to policy files
        policy_files = {
            "Forward": "mini_ars_Forward_policy.npy",
            "Backward": "mini_ars_Backward_policy.npy",
            "Left": "mini_ars_Left_policy.npy",
            "CW": "mini_ars_CW_policy.npy",
            "CCW": "mini_ars_CCW_policy.npy",
        }

        for motion_name, filename in policy_files.items():
            filepath = os.path.join(policies_dir, filename)
            if os.path.exists(filepath):
                try:
                    policy = Policy(state_dim=12, action_dim=14)
                    policy.theta = np.load(filepath)
                    policy.episode_steps = np.inf
                    self.policies[motion_name] = policy
                    self.get_logger().info("Loaded policy: {} -> {}".format(motion_name, filename))
                except Exception as e:
                    self.get_logger().error("Failed to load policy {}: {}".format(filename, e))
            else:
                self.get_logger().warn("Policy file not found: {}".format(filepath))

        # Initialize normalizer
        self.normalizer = Normalizer(state_dim=12)

        self.action = np.zeros(14)
        if self.actions_to_filter > 0:
            self.old_act = self.action[:self.actions_to_filter]
        else:
            self.old_act = self.action.copy()

    def select_policy(self, motion):
        """Select policy based on motion command."""
        if motion in self.policies:
            if self.current_policy_name != motion:
                self.current_policy_name = motion
                self.policy = self.policies[motion]
                self.get_logger().info("Switched to policy: {}".format(motion))
        else:
            if self.current_policy_name != "Stop":
                self.current_policy_name = "Stop"
                self.policy = None
                self.get_logger().info("No policy for motion '{}', using Stop".format(motion))

    def imu_cb(self, msg):
        """Reads the IMU."""
        try:
            self.imu = [
                msg.roll, msg.pitch,
                np.radians(msg.gyro_x),
                np.radians(msg.gyro_y),
                np.radians(msg.gyro_z),
                msg.acc_x, msg.acc_y, msg.acc_z - 9.81
            ]
            self.get_logger().debug(str(msg))
        except Exception as e:
            self.get_logger().error("IMU callback error: {}".format(e))

    def cnt_cb(self, msg):
        """Reads the Contact Sensors."""
        try:
            self.contacts = [msg.fl, msg.fr, msg.bl, msg.br]
            self.get_logger().debug(str(msg))
        except Exception as e:
            self.get_logger().error("Contact callback error: {}".format(e))

    def cmd_cb(self, msg):
        """Reads the desired MiniCmd and selects appropriate policy."""
        try:
            self.mini_cmd = msg
            # Select policy based on motion command
            if self.Agent and msg.motion != "Stop":
                self.select_policy(msg.motion)
            elif msg.motion == "Stop":
                self.select_policy("Stop")
            self.get_logger().debug(str(msg))
        except Exception as e:
            self.get_logger().error("Cmd callback error: {}".format(e))

    def jb_cb(self, msg):
        """Reads the additional joystick buttons."""
        try:
            self.jb = msg
            self.get_logger().debug(str(msg))
        except Exception as e:
            self.get_logger().error("JoyButtons callback error: {}".format(e))

    def move(self):
        """Turn joystick inputs into commands. Uses policy if Agent=True."""
        # Move Type
        if self.mini_cmd.movement == "Stepping":
            step_or_view = False
        else:
            step_or_view = True

        if self.mini_cmd.motion != "Stop":
            self.StepVelocity = copy.deepcopy(self.BaseStepVelocity)
            self.SwingPeriod = np.clip(
                copy.deepcopy(self.BaseSwingPeriod) +
                (-self.mini_cmd.faster + -self.mini_cmd.slower) * self.SV_SCALE,
                self.SwingPeriod_LIMITS[0], self.SwingPeriod_LIMITS[1])

            if self.mini_cmd.movement == "Stepping":
                StepLength = self.mini_cmd.x_velocity + abs(
                    self.mini_cmd.y_velocity * 0.66)
                StepLength = np.clip(StepLength, -1.0, 1.0)
                StepLength *= self.STEPLENGTH_SCALE
                LateralFraction = self.mini_cmd.y_velocity * np.pi / 2
                YawRate = self.mini_cmd.rate * self.YAW_SCALE
                pos = np.array([0.0, 0.0, 0.0])
                orn = np.array([0.0, 0.0, 0.0])
            else:
                StepLength = 0.0
                LateralFraction = 0.0
                YawRate = 0.0
                self.ClearanceHeight = copy.deepcopy(self.BaseClearanceHeight)
                self.PenetrationDepth = copy.deepcopy(self.BasePenetrationDepth)
                self.StepVelocity = copy.deepcopy(self.BaseStepVelocity)
                pos = np.array([0.0, 0.0, self.mini_cmd.z * self.Z_SCALE_CTRL])
                orn = np.array([
                    self.mini_cmd.roll * self.RPY_SCALE,
                    self.mini_cmd.pitch * self.RPY_SCALE,
                    self.mini_cmd.yaw * self.RPY_SCALE
                ])
        else:
            StepLength = 0.0
            LateralFraction = 0.0
            YawRate = 0.0
            self.ClearanceHeight = self.BaseClearanceHeight
            self.PenetrationDepth = self.BasePenetrationDepth
            self.StepVelocity = self.BaseStepVelocity
            self.SwingPeriod = self.BaseSwingPeriod
            pos = np.array([0.0, 0.0, 0.0])
            orn = np.array([0.0, 0.0, 0.0])

        # TODO: integrate into controller
        self.ClearanceHeight += self.jb.updown * self.CHPD_SCALE
        self.PenetrationDepth += self.jb.leftright * self.CHPD_SCALE

        # Manual Reset
        if self.jb.left_bump or self.jb.right_bump:
            self.ClearanceHeight = copy.deepcopy(self.BaseClearanceHeight)
            self.PenetrationDepth = copy.deepcopy(self.BasePenetrationDepth)
            self.StepVelocity = copy.deepcopy(self.BaseStepVelocity)
            self.SwingPeriod = copy.deepcopy(self.BaseSwingPeriod)

        # AGENT: Use pretrained policy to modify foot trajectories
        if self.Agent and self.mini_cmd.motion != "Stop" and self.policy is not None:
            phases = copy.deepcopy(self.bzg.Phases)
            state = []
            # IMU: r, p, gx, gy, gz, ax, ay, az (8)
            state.extend(self.imu)
            # Phases: FL, FR, BL, BR (4)
            state.extend(phases)
            # Contacts: FL, FR, BL, BR (4) - if enabled
            if self.enable_contact:
                state.extend(self.contacts)

            self.normalizer.observe(state)
            # Don't normalize contacts
            if self.enable_contact:
                state[:-4] = self.normalizer.normalize(state)[:-4]
            else:
                state = self.normalizer.normalize(state)

            self.action = self.policy.evaluate(state, None, None)
            self.action = np.tanh(self.action)

            # EXP FILTER
            if self.actions_to_filter > 0:
                self.action[:self.actions_to_filter] = self.alpha * self.old_act + (
                    1.0 - self.alpha) * self.action[:self.actions_to_filter]
                self.old_act = self.action[:self.actions_to_filter]
            else:
                self.action = self.alpha * self.old_act + (1.0 - self.alpha) * self.action
                self.old_act = self.action

            self.ClearanceHeight += self.action[0] * self.CD_SCALE

        # Time
        now = self.get_clock().now()
        dt = (now - self.time).nanoseconds / 1e9
        self.time = now

        # Update Step Period
        self.bzg.Tswing = self.SwingPeriod

        # CLIP
        self.ClearanceHeight = np.clip(self.ClearanceHeight,
                                       self.ClearanceHeight_LIMITS[0],
                                       self.ClearanceHeight_LIMITS[1])
        self.PenetrationDepth = np.clip(self.PenetrationDepth,
                                        self.PenetrationDepth_LIMITS[0],
                                        self.PenetrationDepth_LIMITS[1])

        self.T_bf = self.bzg.GenerateTrajectory(
            StepLength, LateralFraction, YawRate, self.StepVelocity,
            self.T_bf0, self.T_bf, self.ClearanceHeight, self.PenetrationDepth,
            self.contacts, dt)

        T_bf_copy = copy.deepcopy(self.T_bf)

        # AGENT: Apply residuals to foot positions
        if self.Agent and self.mini_cmd.motion != "Stop" and self.policy is not None:
            self.action[2:] *= self.RESIDUALS_SCALE
            T_bf_copy["fl"][:3, 3] += self.action[2:5]
            T_bf_copy["fr"][:3, 3] += self.action[5:8]
            T_bf_copy["bl"][:3, 3] += self.action[8:11]
            T_bf_copy["br"][:3, 3] += self.action[11:14]
            pos[2] += abs(self.action[1]) * self.Z_SCALE

        joint_angles = self.spot.IK(orn, pos, T_bf_copy)

        ja_msg = JointAngles()

        ja_msg.fls = np.degrees(joint_angles[0][0])
        ja_msg.fle = np.degrees(joint_angles[0][1])
        ja_msg.flw = np.degrees(joint_angles[0][2])

        ja_msg.frs = np.degrees(joint_angles[1][0])
        ja_msg.fre = np.degrees(joint_angles[1][1])
        ja_msg.frw = np.degrees(joint_angles[1][2])

        ja_msg.bls = np.degrees(joint_angles[2][0])
        ja_msg.ble = np.degrees(joint_angles[2][1])
        ja_msg.blw = np.degrees(joint_angles[2][2])

        ja_msg.brs = np.degrees(joint_angles[3][0])
        ja_msg.bre = np.degrees(joint_angles[3][1])
        ja_msg.brw = np.degrees(joint_angles[3][2])

        # Move Type
        ja_msg.step_or_view = step_or_view

        self.ja_pub.publish(ja_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpotCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()