#!/usr/bin/env python3
import rclpy
import ctypes

from std_msgs.msg import Int64MultiArray 
from math import sin, cos, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from tf2_ros import TransformBroadcaster  
from geometry_msgs.msg import TransformStamped                  #bieu dien moi quan he giua ca tf frame
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient
from sensor_msgs.msg import JointState



class modbus_node(Node):
    def __init__(self):
        super().__init__("modbus_rtu_node")

        self.data_encoder = []

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.initial_right_encoder = 0
        self.initial_left_encoder = 0
        self.current_right_encoder = 0
        self.current_left_encoder = 0 
        self.prev_right_encoder = 0
        self.prev_left_encoder = 0

        self.center_robot_vel = 0
        self.center_robot_angle_vel = 0

        self.wheel_right_vel = 0
        self.wheel_left_vel = 0
        self.angle_right_position = 0
        self.angle_left_position = 0

        self.radius_wheel = 0.07
        self.distance_two_wheel = 0.315
        self.encoder_pulse = 1000
        self.ratio = 0.5

        self.current_time = 0
        self.past_time = 0

        self.encoder_flag = 1

        self.max_rad_vel = 5.0
        self.analog_max = 2000

        self.encoder_right_raw = 0
        self.encoder_left_raw = 0

        self.prev_right_raw = 0
        self.prev_left_raw = 0

        self.total_right_encoder = 0
        self.total_left_encoder = 0
        self.encoder_max = 65536

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


        self.encoder_publisher = self.create_publisher(Int64MultiArray, "encoder_data", 10)
        self.joint_state_publisher = self.create_publisher(JointState, "/joint_states",10)
        self.control_robot = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)                 # cmd_vel_out
        # self.cmdVelPublisher = self.create_publisher(Twist,'/cmd_vel', 10)

        self.timer1 = self.create_timer(0.1, self.read_register)                    # doc gia tri xung

        self.encoder_subcriber = self.create_subscription(Int64MultiArray, "encoder_data", self.process_data_callback, 10)

        self.timer2 = self.create_timer(0.1, self.caculate_position)                # tinh vi tri robot

        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self.odom_broadcaster = TransformBroadcaster(self)

        self.odomFrameName = "odom"  # Hệ quy chiếu gốc
        self.baseFrameName = "base_link"  # Hệ quy chiếu của robot

        
    def read_register(self):
        # doc gia tri thanh ghi plc
        self.right_encoder = self.client.read_holding_registers(2, count=2, slave=self.slave_id)
        self.left_encoder = self.client.read_holding_registers(0, count=2, slave=self.slave_id)

        # lay gia tri thanh ghi dau tien tu plc
        self.value_right_encoder = self.right_encoder.registers[0]
        self.value_left_encoder = self.left_encoder.registers[0]
        # self.get_logger().info(f"Encode R: {self.value_right_encoder} L: {self.value_left_encoder}")

        delta_right = self.value_right_encoder - self.prev_right_raw
        if delta_right < -self.encoder_max / 2:
            delta_right += self.encoder_max
        elif delta_right > self.encoder_max / 2:
            delta_right -= self.encoder_max
        self.total_right_encoder += delta_right
        self.prev_right_raw = self.value_right_encoder

        delta_left = self.value_left_encoder - self.prev_left_raw
        if delta_left < -self.encoder_max / 2:
            delta_left += self.encoder_max
        elif delta_left > self.encoder_max / 2:
            delta_left -= self.encoder_max
        self.total_left_encoder += delta_left
        self.prev_left_raw = self.value_left_encoder

        
        # chuyen thanh kieu int16
        # self.value_right_encoder_abc = ctypes.c_int64(self.value_right_encoder).value
        # self.value_left_encoder_abc = ctypes.c_int64(self.value_left_encoder).value

        self.data_encoder = [self.total_right_encoder, self.total_left_encoder]
        msg =  Int64MultiArray()
        msg.data = [self.total_right_encoder, self.total_left_encoder]

        # self.get_logger().info(f"Encode R: {self.total_right_encoder} L: {self.total_left_encoder}")
        
        self.encoder_publisher.publish(msg)

        if self.right_encoder.isError() or self.left_encoder.isError():
            self.get_logger().error("Lỗi khi doc PLC")
        else:
            pass
            # self.get_logger().info("Đã doc registers")


    def process_data_callback(self, msg:Int64MultiArray):
        if(self.encoder_flag == 1):
            self.initial_right_encoder = msg.data[0]
            self.initial_left_encoder = msg.data[1]
            self.encoder_flag = 0
        else:
            self.current_right_encoder = msg.data[0] - self.initial_right_encoder
            self.current_left_encoder = msg.data[1] - self.initial_left_encoder

            # print(self.current_right_encoder, self.current_left_encoder)
            
            
    def publish_joint_state(self):
        # Tạo thông điệp JointState
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.angle_left_position, self.angle_right_position]  # Vị trí (rad)
        joint_state.velocity = [self.wheel_left_vel, self.wheel_right_vel]  # Vận tốc (m/s)
        joint_state.effort = []

        # Xuất bản thông điệp JointState
        self.joint_state_publisher.publish(joint_state)

    def caculate_position(self):

        now = self.get_clock().now().to_msg()
        self.current_time = now.sec * 1_000_000_000 + now.nanosec  
                                              
        # self.get_logger().info(f"TIME: {self.current_time}")
        if(int(self.current_time) - self.past_time > 0):
            delta_t = (self.current_time - self.past_time) * 1e-9
            # self.get_logger().info(f"DELTA T: {delta_t}")

            # tinh do chenh lech goc
            angle_right_change = (self.current_right_encoder - self.prev_right_encoder) * ((2 * pi) / (self.encoder_pulse )) * self.ratio   # rad
            angle_left_change = (self.current_left_encoder - self.prev_left_encoder) * ((2 * pi) / (self.encoder_pulse )) * self.ratio

            # self.get_logger().info(f"Encode Right: {self.current_right_encoder} Left: {self.current_left_encoder}")

            angle_right_vel = angle_right_change / delta_t                                                                             # rad/ms           
            angle_left_vel = angle_left_change / delta_t

            self.wheel_right_vel = angle_right_vel * self.radius_wheel
            self.wheel_left_vel = angle_left_vel * self.radius_wheel

            # vel robot
            self.center_robot_vel = (self.wheel_right_vel + self.wheel_left_vel) / 2
            self.center_robot_angle_vel= (self.wheel_right_vel - self.wheel_left_vel) / self.distance_two_wheel

            distance_change = self.center_robot_vel * delta_t
            angle_change = self.center_robot_angle_vel * delta_t

            self.x = self.x + distance_change * cos(self.theta) 
            self.y = self.y + distance_change * sin(self.theta)
            self.theta = self.theta + angle_change

            self.angle_left_position += angle_left_vel * delta_t               # goc quay
            self.angle_right_position += angle_right_vel * delta_t

            # self.get_logger().info("tinh toan thanh cong")
            # self.get_logger().info(f"Encode R: {self.current_right_encoder} L: {self.current_left_encoder}")
            # self.get_logger().info(f"Theta: {self.theta}")

        odometry = Odometry()
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.header.frame_id = "odom"

        # Thiết lập vị trí
        odometry.pose.pose.position.x = self.x
        odometry.pose.pose.position.y = self.y
        odometry.pose.pose.position.z = 0.0

        # Chuyển đổi góc quay sang quaternion (vi ros dung quaternion de bieu dien goc quay, chu khong dung theta truc z cua odometry)
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.theta / 2)
        quaternion.w = cos(self.theta / 2)
        odometry.pose.pose.orientation = quaternion

        # Thiết lập vận tốc
        odometry.twist.twist.linear.x = self.center_robot_vel
        odometry.twist.twist.angular.z = self.center_robot_angle_vel

        # Publish Odometry
        self.odom_publisher.publish(odometry)
        self.publish_joint_state()
                
        # cmd_vel_msg = Twist()
        # cmd_vel_msg.linear.x = self.center_robot_vel  
        # cmd_vel_msg.angular.z = self.center_robot_angle_vel  
        # self.cmdVelPublisher.publish(cmd_vel_msg)

        # print(cmd_vel_msg)

        self.past_time = self.current_time
        self.prev_right_encoder = self.current_right_encoder
        self.prev_left_encoder = self.current_left_encoder



        ###################### CONTROL ######################
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

        # print(analog_r, analog_l)

  
        self.send_to_motor(analog_r, analog_l)


    def send_to_motor(self, analog_r, analog_l):
        try:
            self.client.write_coil(address=3, value=True, slave=self.slave_id)

            # chieu banh phai (M4)
            if analog_r >= 0:
                self.client.write_coil(address=4, value=False, slave=self.slave_id)  # Đi tới
            else:
                self.client.write_coil(address=4, value=True, slave=self.slave_id)   # Đi lùi

            # chieu banh trai (M5)
            if analog_l >= 0:
                self.client.write_coil(address=5, value=True, slave=self.slave_id)   # Đi tới
            else:
                self.client.write_coil(address=5, value=False, slave=self.slave_id)  # Đi lùi

            self.client.write_register(101, ctypes.c_int16(abs(analog_r)).value, slave=self.slave_id)  # banh phai
            self.client.write_register(100, ctypes.c_int16(abs(analog_l)).value, slave=self.slave_id)  # banh trai
            self.get_logger().info(f"Send Vel - R: {analog_r}, L: {analog_l}")
        
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


        
def main(args=None):
    rclpy.init(args=args)
    node = modbus_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
