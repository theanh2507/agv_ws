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
            self.get_logger().info("Kết nối Modbus thành công")
        else:
            self.get_logger().error("Không thể kết nối với thiết bị Modbus")

        # Nhận lệnh từ /cmd_vel
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Tính vận tốc bánh (m/s)
        v_r = (2 * v + w * self.distance_two_wheel) / 2
        v_l = (2 * v - w * self.distance_two_wheel) / 2

        # Chuyển sang tốc độ góc (rad/s), rồi sang xung/chu kỳ
        angular_r = v_r / self.radius_wheel
        angular_l = v_l / self.radius_wheel

        pulses_r = int((angular_r * self.encoder_pulse) / (2 * pi * self.ratio))
        pulses_l = int((angular_l * self.encoder_pulse) / (2 * pi * self.ratio))

        self.send_to_motor(pulses_r, pulses_l)

    def send_to_motor(self, pulses_r, pulses_l):
        try:
            self.client.write_register(0, ctypes.c_int16(pulses_l).value, slave=self.slave_id)  # bánh trái
            self.client.write_register(2, ctypes.c_int16(pulses_r).value, slave=self.slave_id)  # bánh phải
            self.get_logger().info(f"Gửi vận tốc - L: {pulses_l}, R: {pulses_r}")
        except Exception as e:
            self.get_logger().error(f"Lỗi khi gửi dữ liệu tới motor: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
