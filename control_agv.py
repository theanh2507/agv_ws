import rclpy
import struct
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient
from std_msgs.msg import Int32MultiArray 

import ctypes

class modbus_node(Node):
    def __init__(self):
        super().__init__("modbus_rtu_node")

        # self.data = [0]*10
        self.data_encoder = []

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("slave_id", 3)
        self.declare_parameter("parity", "N")

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.parity = self.get_parameter("parity").get_parameter_value().string_value
        self.slave_id = self.get_parameter("slave_id").get_parameter_value().integer_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value

        self.client = ModbusSerialClient(framer='rtu',port=self.port, baudrate=self.baudrate, 
                                         parity='N', bytesize=8, stopbits=1, timeout=1)
        
        if self.client.connect():
            self.get_logger().info("ket noi thanh cong")
        else:
            self.get_logger().info("khong the ket noi")

        
    #     # Tạo Subscriber để nhận dữ liệu
    #     self.create_subscription(Int32MultiArray, 'topic', self.write_register_callback, 10)

    #     # Tạo Timer để tăng giá trị và ghi vào PLC mỗi 1 giây
        self.timer = self.create_timer(1.0, self.read_register)

    # def write_register_callback(self, msg):
    #         """Cập nhật giá trị khi nhận dữ liệu từ ROS2"""
    #         self.data = msg.data  # Ghi đè dữ liệu nhận được
    #         self.get_logger().info(f"Cập nhật giá trị từ ROS2: {self.data}")

    # def increment_and_write_registers(self):
    #         """Tăng giá trị và ghi xuống PLC mỗi 1 giây"""
    #         self.data = [x + 1 for x in self.data]  # Mỗi phần tử trong mảng +1
    #         result = self.client.write_registers(30, self.data, slave=self.slave_id)
            
    #         if result.isError():
    #             self.get_logger().error("Lỗi khi ghi vào PLC")
    #         else:
    #             self.get_logger().info(f"Đã ghi registers: {self.data}")

    def read_register(self):
        # doc gia trij thanh ghi plc
        self.right_encoder = self.client.read_holding_registers(0, count=2, slave=self.slave_id)
        self.left_encoder = self.client.read_holding_registers(2, count=2, slave=self.slave_id)

        # lay gia tri thanh ghi dau tien tu plc
        self.value_right_encoder = self.right_encoder.registers[0]
        self.value_left_encoder = self.left_encoder.registers[0]

        # chuyen thanh kieu int64
        self.value_right_encoder = ctypes.c_int64(self.value_right_encoder).value
        self.value_left_encoder = ctypes.c_int64(self.value_left_encoder).value

        self.data_encoder = [self.value_right_encoder, self.value_left_encoder]

        if self.right_encoder.isError() or self.left_encoder.isError():
            self.get_logger().error("Lỗi khi doc PLC")
            
        else:
            self.get_logger().info("Đã doc registers")
            print(self.data_encoder)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = modbus_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
