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

        self.max_rad_vel = 4.1666667                        # linear.x = 0.5
        self.analog_max = 2000

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

    
    def map_rad_s_to_analog(self, rad_s, max_rad_vel, analog_max):
        # Giới hạn rad_s trong [-max_rad_s, max_rad_s]
        rad_s = max(min(rad_s, max_rad_vel), -max_rad_vel)

        analog_value = int((rad_s / max_rad_vel) * analog_max)

        return analog_value

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # van toc banh (m/s)
        v_r = v + (w * self.distance_two_wheel / 2)
        v_l = v - (w * self.distance_two_wheel / 2)

        # van toc goc banh
        angular_r = v_r / self.radius_wheel             # (rad/s)
        angular_l = v_l / self.radius_wheel

        analog_r = self.map_rad_s_to_analog(angular_r, self.max_rad_vel, self.analog_max)
        analog_l = self.map_rad_s_to_analog(angular_l, self.max_rad_vel, self.analog_max)

        angular_r_rpm = int((angular_r * 60) / (2 * pi))
        angular_l_rpm = int((angular_l * 60) / (2 * pi))

        print(analog_r, analog_l)

  
        self.send_to_motor(analog_r, analog_l)


    def send_to_motor(self, analog_r, analog_l):
        try:
            self.client.write_coil(address=3, value=True, slave=self.slave_id)

            # Chiều bánh phải (M4)
            if analog_r >= 0:
                self.client.write_coil(address=4, value=False, slave=self.slave_id)  # Đi tới
            else:
                self.client.write_coil(address=4, value=True, slave=self.slave_id)   # Đi lùi

            # Chiều bánh trái (M5)
            if analog_l >= 0:
                self.client.write_coil(address=5, value=True, slave=self.slave_id)   # Đi tới
            else:
                self.client.write_coil(address=5, value=False, slave=self.slave_id)  # Đi lùi

            self.client.write_register(101, ctypes.c_int16(abs(analog_r)).value, slave=self.slave_id)  # banh phai
            self.client.write_register(100, ctypes.c_int16(abs(analog_l)).value, slave=self.slave_id)  # banh trai
            self.get_logger().info(f"Send Vel - L: {analog_l}, R: {analog_r}")
        
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
