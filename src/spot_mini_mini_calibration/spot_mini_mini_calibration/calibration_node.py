#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from spot_mini_mini_interfaces.msg import JointPulse
from spot_mini_mini_interfaces.srv import CalibServo


class ServoCalibrator(Node):
    def __init__(self):
        super().__init__('servo_calibrator')
        self.srv = self.create_service(
            CalibServo, 'servo_calibrator', self.calib_service_cb)
        self.jp_pub = self.create_publisher(
            JointPulse, 'spot/pulse', 10)
        self.get_logger().info(
            'ServoCalibrator ready. Pulse width in µs, nominal ~500–2500.')

    def calib_service_cb(self, request, response):
        try:
            msg = JointPulse()
            msg.servo_num = request.servo_num
            msg.servo_pulse = request.servo_pulse
            self.jp_pub.publish(msg)
            response.response = 'Servo Command Sent.'
            self.get_logger().info(
                f'Published pulse {msg.servo_pulse} µs to servo {msg.servo_num}')
        except Exception as e:
            self.get_logger().error(f'Failed: {e}')
            response.response = 'FAILED to send Servo Command'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServoCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
