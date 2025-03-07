
import rclpy
import math
from myimu.msg import ImuSensor
from rclpy.node import Node
from myimu.msg import Calculation
import numpy as np

class Magnetic_ya(Node):
    def __init__(self):
        super().__init__('imu_orientation_calculator3')

        self.imu_sub = self.create_subscription(ImuSensor, '/imu/sensor', self.imu_callback, 10)

        self.orientation_pub = self.create_publisher(Calculation, '/calculation/mag', 10)

    def imu_callback(self, mag_msg):
        mag_x = mag_msg.magnetic_field.x
        mag_y = mag_msg.magnetic_field.y
        mag_z = mag_msg.magnetic_field.z

        yaw_rad = np.arctan2(mag_y, mag_x)  # 라디안 값
        yaw_deg = np.degrees(yaw_rad)

        orientation3_msg = Calculation()
        orientation3_msg.magnetic.roll = 0.0
        orientation3_msg.magnetic.pitch = 0.0
        orientation3_msg.magnetic.yaw = yaw_deg

        self.orientation_pub.publish(orientation3_msg)  # ✅ 올바른 변수명 사용


def main():
    rclpy.init()
    node = Magnetic_ya()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
import rclpy
import math
from myimu.msg import ImuSensor, Calculation
from rclpy.node import Node
import numpy as np


class Magnetic_ya(Node):
    def __init__(self):
        super().__init__('imu_orientation_calculator3')

        self.imu_sub = self.create_subscription(ImuSensor, '/imu/sensor', self.imu_callback, 10)
        self.orientation_sub = self.create_subscription(Calculation, '/calculation/final', self.orientation_callback, 10)

        self.orientation_pub = self.create_publisher(Calculation, '/calculation/mag', 10)

        self.mag_x = 0.0
        self.mag_y = 0.0
        self.mag_z = 0.0
        self.roll = 0.0
        self.pitch = 0.0

    def imu_callback(self, mag_msg):
        self.mag_x = mag_msg.magnetic_field.x
        self.mag_y = mag_msg.magnetic_field.y
        self.mag_z = mag_msg.magnetic_field.z


    def orientation_callback(self, orientation_msg):
        self.roll = orientation_msg.final.pitch
        self.pitch = orientation_msg.final.roll
        self.check_and_process_data()

    def check_and_process_data(self):
        mag_x = self.mag_x * np.cos(self.pitch) + self.mag_z * np.sin(self.pitch)
        mag_y = self.mag_x * np.sin(self.roll) * np.sin(self.pitch) + self.mag_y * np.cos(self.roll) - self.mag_z * np.sin(self.roll) * np.cos(self.pitch)

        yaw_rad = np.arctan2(mag_y, mag_x)  # 라디안 값
        yaw_deg = np.degrees(yaw_rad)

        orientation_msg = Calculation()
        orientation_msg.magnetic.roll = 0.0
        orientation_msg.magnetic.pitch = 0.0
        orientation_msg.magnetic.yaw = yaw_deg

        self.orientation_pub.publish(orientation_msg)  # ✅ 올바른 변수명 사용


def main():
    rclpy.init()
    node = Magnetic_ya()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''