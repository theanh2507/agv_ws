#!/usr/bin/env python3
import rclpy
import ctypes
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pymodbus.client import ModbusSerialClient
from math import pi


class MotorControlNode(Node):
    def __init__(self):
        super().__init__("motor_control_node")

        # Robot parameters
        self.radius_wheel = 0.12
        self.distance_two_wheel = 0.315
        self.encoder_pulse = 1000
        self.ratio = 0.5

        # Modbus parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("slave_id", 3)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.slave_id = self.get_parameter("slave_id").get_parameter_value().integer_value

        self.client = ModbusSerialClient(
            framer='rtu',
            port=self.port,
            baudrate=self.baudrate,
            parity='N',
            bytesize=8,
            stopbits=1,
            timeout=1
        )

        if self.client.connect():
            self.get_logger().info("Ket noi thanh cong")
        else:
            self.get_logger().error("Ket noi that bai")

        # Nhận lệnh từ /cmd_vel
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Tính vận tốc bánh (m/s)
        v_r = v + (w * self.distance_two_wheel / 2)
        v_l = v - (w * self.distance_two_wheel / 2)

        # Chuyển sang tốc độ góc (rad/s), rồi sang xung/chu kỳ
        angular_r = v_r / self.radius_wheel
        angular_l = v_l / self.radius_wheel

        angular_r_rpm = (angular_r * 60) / (2 * pi)
        angular_l_rpm = (angular_l * 60) / (2 * pi)

        pulses_r = int((angular_r * self.encoder_pulse) / (2 * pi * self.ratio))
        pulses_l = int((angular_l * self.encoder_pulse) / (2 * pi * self.ratio))

        self.send_to_motor(angular_r_rpm, angular_l_rpm)

    def send_to_motor(self, angular_r_rpm, angular_l_rpm):
        try:
            self.client.write_register(0, ctypes.c_int16(angular_r_rpm).value, slave=self.slave_id)  # banh phai
            self.client.write_register(2, ctypes.c_int16(angular_l_rpm).value, slave=self.slave_id)  # banh trai
            self.get_logger().info(f"Send Vel - L: {angular_l_rpm}, R: {angular_r_rpm}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
