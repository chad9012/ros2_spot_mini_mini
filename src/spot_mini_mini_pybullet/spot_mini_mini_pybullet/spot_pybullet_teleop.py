#!/usr/bin/env python3
"""
spot_pybullet_teleop.py

ROS2 node that drives Spot Mini Mini in PyBullet using /mini_cmd and /joybuttons.
Publishes IMU data on /spot/imu and contact data on /spot/contact.

STATE DIMENSIONS (12):
- IMU: roll, pitch, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z (8)
- Gait phases: FL, FR, BL, BR (4)

ACTION DIMENSIONS (14):
- action[0]: Clearance height adjustment
- action[1]: Body Z height adjustment  
- action[2:5]: FL foot residual (x,y,z)
- action[5:8]: FR foot residual (x,y,z)
- action[8:11]: BL foot residual (x,y,z)
- action[11:14]: BR foot residual (x,y,z)
"""

import rclpy
from rclpy.node import Node
import numpy as np
import copy

from spot_mini_mini_interfaces.msg import MiniCmd, JoyButtons, IMUdata, ContactData

from spot_mini_mini_pybullet.spotmicro.GymEnvs.spot_bezier_env import spotBezierEnv
from spot_mini_mini_pybullet.spotmicro.Kinematics.SpotKinematics import SpotModel
from spot_mini_mini_pybullet.spotmicro.GaitGenerator.Bezier import BezierGait

# Controller Params
STEPLENGTH_SCALE = 0.06
Z_SCALE_CTRL = 0.12
RPY_SCALE = 0.6
SV_SCALE = 0.1
CHPD_SCALE = 0.0005
YAW_SCALE = 1.5


class SpotPyBulletTeleop(Node):
    def __init__(self):
        super().__init__('spot_pybullet_teleop')

        self.declare_parameter('render', False)
        self.declare_parameter('on_rack', False)
        self.declare_parameter('height_field', False)
        self.declare_parameter('draw_foot_path', False)
        self.declare_parameter('seed', 0)

        render = self.get_parameter('render').value
        on_rack = self.get_parameter('on_rack').value
        height_field = self.get_parameter('height_field').value
        draw_foot_path = self.get_parameter('draw_foot_path').value
        seed = self.get_parameter('seed').value

        # Subscribers
        self.sub_cmd = self.create_subscription(
            MiniCmd, 'mini_cmd', self.mini_cmd_cb, 10)
        self.sub_jb = self.create_subscription(
            JoyButtons, 'joybuttons', self.jb_cb, 10)

        # Publishers for IMU and contact data (for policy evaluation)
        self.imu_pub = self.create_publisher(IMUdata, 'spot/imu', 10)
        self.contact_pub = self.create_publisher(ContactData, 'spot/contact', 10)

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

        self.BaseStepVelocity = 0.1
        self.StepVelocity = self.BaseStepVelocity
        self.BaseSwingPeriod = 0.2
        self.SwingPeriod = self.BaseSwingPeriod
        self.BaseClearanceHeight = 0.04
        self.BasePenetrationDepth = 0.005
        self.ClearanceHeight = self.BaseClearanceHeight
        self.PenetrationDepth = self.BasePenetrationDepth

        # For acceleration computation
        self.prev_lin_vel = np.array([0.0, 0.0, 0.0])
        self.prev_ang_vel = np.array([0.0, 0.0, 0.0])

        self.get_logger().info("Loading PyBullet environment...")
        self.env = spotBezierEnv(
            render=render,
            on_rack=on_rack,
            height_field=height_field,
            draw_foot_path=draw_foot_path)

        self.state, _ = self.env.reset(seed=seed)
        np.random.seed(seed)

        self.spot = SpotModel()
        self.dt = self.env._time_step
        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf = copy.deepcopy(self.T_bf0)
        self.bzg = BezierGait(dt=self.env._time_step)

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / 600.0, self.move)

        self.get_logger().info("SpotPyBulletTeleop ready")
        self.get_logger().info("Publishing IMU on /spot/imu")
        self.get_logger().info("Publishing contacts on /spot/contact")

    def mini_cmd_cb(self, msg):
        self.mini_cmd = msg

    def jb_cb(self, msg):
        self.jb = msg

    def publish_sensors(self):
        """Read IMU and contact data from PyBullet and publish."""
        # Get base orientation (quaternion -> Euler)
        orn = self.env.spot.GetBaseOrientation()
        roll, pitch, yaw = self.env._pybullet_client.getEulerFromQuaternion(
            [orn[0], orn[1], orn[2], orn[3]])

        # Get angular velocity (rad/s from PyBullet)
        ang_vel = self.env.spot.prev_ang_twist  # [wx, wy, wz]

        # Get linear velocity for acceleration computation
        lin_vel = self.env.spot.prev_lin_twist  # [vx, vy, vz]

        # Compute linear acceleration from velocity difference
        # a = (v_current - v_prev) / dt
        dt = 1.0 / 600.0  # Control loop dt
        lin_acc = (lin_vel - self.prev_lin_vel) / dt
        # Add gravity compensation (PyBullet world frame has z-up)
        lin_acc[2] += 9.81

        # Store for next iteration
        self.prev_lin_vel = np.array(lin_vel)
        self.prev_ang_vel = np.array(ang_vel)

        # Publish IMU
        imu_msg = IMUdata()
        imu_msg.roll = float(roll)
        imu_msg.pitch = float(pitch)
        # imu_msg.yaw = float(yaw)
        # Convert rad/s to deg/s for IMU message convention
        imu_msg.gyro_x = float(np.degrees(ang_vel[0]))
        imu_msg.gyro_y = float(np.degrees(ang_vel[1]))
        imu_msg.gyro_z = float(np.degrees(ang_vel[2]))
        imu_msg.acc_x = float(lin_acc[0])
        imu_msg.acc_y = float(lin_acc[1])
        imu_msg.acc_z = float(lin_acc[2])
        self.imu_pub.publish(imu_msg)

        # Get contact states from observation
        # The observation from spot_bezier_env includes contact info
        # contacts = [FL_contact, FR_contact, BL_contact, BR_contact]
        # These are typically the last 4 elements of the observation
        contacts = self.state[-4:] if len(self.state) >= 4 else [0, 0, 0, 0]

        contact_msg = ContactData()
        # Threshold contact values (observation may be continuous, binarize)
        contact_msg.fl = bool(contacts[0] > 0.5) if len(contacts) > 0 else False
        contact_msg.fr = bool(contacts[1] > 0.5) if len(contacts) > 1 else False
        contact_msg.bl = bool(contacts[2] > 0.5) if len(contacts) > 2 else False
        contact_msg.br = bool(contacts[3] > 0.5) if len(contacts) > 3 else False
        self.contact_pub.publish(contact_msg)

    def move(self):
        if self.mini_cmd.motion != "Stop":
            self.StepVelocity = self.BaseStepVelocity
            self.SwingPeriod = np.clip(
                self.BaseSwingPeriod +
                (-self.mini_cmd.faster + -self.mini_cmd.slower) * SV_SCALE,
                0.1, 0.3)

            if self.mini_cmd.movement == "Stepping":
                StepLength = self.mini_cmd.x_velocity + abs(
                    self.mini_cmd.y_velocity * 0.66)
                StepLength = np.clip(StepLength, -1.0, 1.0)
                StepLength *= STEPLENGTH_SCALE
                LateralFraction = self.mini_cmd.y_velocity * np.pi / 2
                YawRate = self.mini_cmd.rate * YAW_SCALE
                pos = np.array([0.0, 0.0, self.mini_cmd.z * Z_SCALE_CTRL])
                orn = np.array([0.0, 0.0, 0.0])
            else:
                StepLength = 0.0
                LateralFraction = 0.0
                YawRate = 0.0
                self.ClearanceHeight = self.BaseClearanceHeight
                self.PenetrationDepth = self.BasePenetrationDepth
                self.StepVelocity = self.BaseStepVelocity
                pos = np.array([0.0, 0.0, self.mini_cmd.z * Z_SCALE_CTRL])
                orn = np.array([
                    self.mini_cmd.roll * RPY_SCALE,
                    self.mini_cmd.pitch * RPY_SCALE,
                    self.mini_cmd.yaw * RPY_SCALE])
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

        self.ClearanceHeight += self.jb.updown * CHPD_SCALE
        self.PenetrationDepth += self.jb.leftright * CHPD_SCALE

        if self.jb.left_bump or self.jb.right_bump:
            self.ClearanceHeight = self.BaseClearanceHeight
            self.PenetrationDepth = self.BasePenetrationDepth
            self.StepVelocity = self.BaseStepVelocity
            self.SwingPeriod = self.BaseSwingPeriod
            self.state, _ = self.env.reset()
            # Reset velocity history
            self.prev_lin_vel = np.array([0.0, 0.0, 0.0])
            self.prev_ang_vel = np.array([0.0, 0.0, 0.0])

        contacts = self.state[-4:]

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        self.bzg.Tswing = self.SwingPeriod
        self.T_bf = self.bzg.GenerateTrajectory(
            StepLength, LateralFraction, YawRate,
            self.StepVelocity, self.T_bf0, self.T_bf,
            self.ClearanceHeight, self.PenetrationDepth,
            contacts, dt)

        joint_angles = self.spot.IK(orn, pos, self.T_bf)
        self.env.pass_joint_angles(joint_angles.reshape(-1))

        action = self.env.action_space.sample()
        action[:] = 0.0
        self.state, reward, terminated, truncated, _ = self.env.step(action)

        # Publish sensor data after simulation step
        self.publish_sensors()

        if terminated or truncated:
            self.get_logger().info("Episode terminated, resetting...")
            self.state, _ = self.env.reset()
            self.prev_lin_vel = np.array([0.0, 0.0, 0.0])
            self.prev_ang_vel = np.array([0.0, 0.0, 0.0])


def main(args=None):
    rclpy.init(args=args)
    node = SpotPyBulletTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()