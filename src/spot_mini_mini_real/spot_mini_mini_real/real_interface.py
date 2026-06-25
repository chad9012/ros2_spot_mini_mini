#!/usr/bin/env python3
"""
real_interface.py — ROS2 ↔ Teensy serial bridge

Subscribes: /spot/joint_trajectory_controller/joint_trajectory
Publishes:  /spot/contact (ContactData), /spot/imu (IMUdata)

Teensy Protocol:
  Send: JOINTS:12_angles_in_degrees,step_or_view\n
  Recv: CONTACT:fl,fr,bl,br\n
  Recv: IMU:roll,pitch,gx,gy,gz,ax,ay,az\n
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from spot_mini_mini_interfaces.msg import ContactData, IMUdata
import serial
import numpy as np


class RealInterface(Node):
    def __init__(self):
        super().__init__('real_interface')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyS0')
        self.declare_parameter('baudrate', 115200)  # Teensy uses 115200

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value

        # Serial
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info('Serial opened: %s @ %d baud' % (port, baud))
        except serial.SerialException as e:
            self.get_logger().fatal('Failed to open serial: %s' % str(e))
            raise

        # Publishers
        self.contact_pub = self.create_publisher(ContactData, 'spot/contact', 10)
        self.imu_pub = self.create_publisher(IMUdata, 'spot/imu', 10)

        # Subscriber
        self.sub = self.create_subscription(
            JointTrajectory,
            '/spot/joint_trajectory_controller/joint_trajectory',
            self.trajectory_cb,
            10
        )

        # Joint order must match Teensy expectation
        self.joint_names = [
            'motor_front_left_hip', 'motor_front_left_upper_leg', 'motor_front_left_lower_leg',
            'motor_front_right_hip', 'motor_front_right_upper_leg', 'motor_front_right_lower_leg',
            'motor_back_left_hip', 'motor_back_left_upper_leg', 'motor_back_left_lower_leg',
            'motor_back_right_hip', 'motor_back_right_upper_leg', 'motor_back_right_lower_leg',
        ]

        self.latest_angles = [0.0] * 12
        self.step_or_view = 0.0  # 0.0 = stepping, 1.0 = viewing

        # Timer: send to Teensy at 50 Hz
        self.timer = self.create_timer(0.02, self.write_serial)

        # Timer: read from Teensy at 100 Hz
        self.read_timer = self.create_timer(0.01, self.read_serial)

        self.get_logger().info('Real interface ready.')

    def trajectory_cb(self, msg):
        if not msg.points:
            return
        point = msg.points[0]
        name_to_idx = {n: i for i, n in enumerate(msg.joint_names)}
        for i, name in enumerate(self.joint_names):
            if name in name_to_idx:
                self.latest_angles[i] = point.positions[name_to_idx[name]]
        # step_or_view is not in JointTrajectory, default to 0 (stepping)
        # If you need viewing mode, we can add a separate subscriber or infer from mini_cmd

    def write_serial(self):
        """Send JOINTS: command to Teensy."""
        # Convert radians → degrees (Teensy expects degrees)
        deg = np.degrees(self.latest_angles)
        vals = ','.join(['%.2f' % a for a in deg])
        packet = 'JOINTS:%s,%.1f\n' % (vals, self.step_or_view)
        try:
            self.ser.write(packet.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().warn('Serial write failed: %s' % str(e))

    def read_serial(self):
        """Parse CONTACT: and IMU: lines from Teensy."""
        while self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
            except Exception:
                continue

            if line.startswith('CONTACT:'):
                parts = line[8:].split(',')
                if len(parts) >= 4:
                    msg = ContactData()
                    msg.fl = int(parts[0]) != 0
                    msg.fr = int(parts[1]) != 0
                    msg.bl = int(parts[2]) != 0
                    msg.br = int(parts[3]) != 0
                    self.contact_pub.publish(msg)

            elif line.startswith('IMU:'):
                parts = line[4:].split(',')
                if len(parts) >= 8:
                    msg = IMUdata()
                    msg.roll = float(parts[0])
                    msg.pitch = float(parts[1])
                    msg.gyro_x = float(parts[2])
                    msg.gyro_y = float(parts[3])
                    msg.gyro_z = float(parts[4])
                    msg.acc_x = float(parts[5])
                    msg.acc_y = float(parts[6])
                    msg.acc_z = float(parts[7])
                    self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RealInterface()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()