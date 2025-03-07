'''
import rclpy
from rclpy.node import Node
from  myimu.msg import Calculation, ImuSensor


class Kalmanfiltter(Node):
    def __init__(self):
        super().__init__('kalman_filtter')
        self.imu_sub = self.create_subscription(Calculation, '/calculation/accel', self.imu_callback, 10)
        self.kalman_pub = self.create_publisher(Calculation, '/calculation/final', 10)

    def imu_callback(self, accel_msg):
        kalman_msg = Calculation()
        kalman_msg.final.roll = accel_msg.accel.roll
        kalman_msg.final.pitch = accel_msg.accel.pitch
        kalman_msg.final.yaw = accel_msg.accel.yaw

        self.kalman_pub.publish(kalman_msg)  # ✅ 올바른 변수명 사용

def main():
    rclpy.init()
    node = Kalmanfiltter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
'''
import rclpy
from rclpy.node import Node
from myimu.msg import ImuSensor, Calculation
from geometry_msgs.msg import Quaternion
import numpy as np
import math
from filterpy.kalman import KalmanFilter #pip install filterpy

class IMUFusion(Node):
    def __init__(self):
        super().__init__('imu_fusion')

        # ✅ IMU 데이터 구독
        self.imu_sub = self.create_subscription(Calculation, '/calculation/accel', self.imu_callback, 10)

        # ✅ 보정된 Roll, Pitch, Yaw 발행
        self.orientation_pub = self.create_publisher(Calculation, '/calculation/final', 10)

        # ✅ Kalman Filter 설정 (상태 벡터: roll, pitch, yaw)
        self.kf = KalmanFilter(dim_x=3, dim_z=3)
        self.kf.x = np.zeros(3)  # 초기 상태 (roll, pitch, yaw)
        self.kf.F = np.eye(3)  # 상태 전이 행렬
        self.kf.H = np.eye(3)  # 측정 행렬
        self.kf.P *= 1.0  # 초기 오차 공분산 행렬
        self.kf.R *= 0.5  # 측정 노이즈 공분산 행렬
        self.kf.Q *= 0.01  # 프로세스 노이즈 공분산 행렬

        self.get_logger().info("✅ IMU Fusion Node Started!")

    def imu_callback(self, msg):
        """ 가속도, 자이로, 자기 센서를 융합하여 보정된 Roll, Pitch, Yaw 계산 """

        # ✅ 가속도 기반 Roll, Pitch 계산
        accel_x, accel_y, accel_z = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        roll_accel = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2))
        pitch_accel = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))

        # ✅ 자이로 적분을 통해 Roll, Pitch, Yaw 계산
        gyro_x, gyro_y, gyro_z = msg.angular_acceleration.x, msg.angular_acceleration.y, msg.angular_acceleration.z
        dt = 0.02  # 50Hz 가정
        roll_gyro = gyro_x * dt
        pitch_gyro = gyro_y * dt
        yaw_gyro = gyro_z * dt

        # ✅ 자기 센서 기반 Yaw 계산
        mag_x, mag_y = msg.magnetic_field.x, msg.magnetic_field.y
        yaw_mag = np.arctan2(mag_y, mag_x)

        # ✅ Kalman Filter 보정
        measured_state = np.array([math.degrees(roll_accel), math.degrees(pitch_accel), math.degrees(yaw_mag)])
        self.kf.predict()
        self.kf.update(measured_state)
        roll, pitch, yaw = self.kf.x

        # ✅ 쿼터니언 변환
        qx, qy, qz, qw = self.euler_to_quaternion(math.radians(roll), math.radians(pitch), math.radians(yaw))

        # ✅ 보정된 Roll, Pitch, Yaw 발행
        orientation_msg = Calculation()
        orientation_msg.accel.roll = roll
        orientation_msg.accel.pitch = pitch
        orientation_msg.accel.yaw = yaw
        orientation_msg.quaternion.x = qx
        orientation_msg.quaternion.y = qy
        orientation_msg.quaternion.z = qz
        orientation_msg.quaternion.w = qw

        self.orientation_pub.publish(orientation_msg)
        self.get_logger().info(f"✅ 보정된 IMU 데이터: Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """ Roll, Pitch, Yaw -> Quaternion 변환 """
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw
'''
'''
import rclpy
from rclpy.node import Node
from myimu.msg import ImuSensor, Calculation
from geometry_msgs.msg import Quaternion
import numpy as np
import math
from filterpy.kalman import KalmanFilter #pip install filterpy

def main():
    rclpy.init()
    node = IMUFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


class IMUFusion(Node):
    def __init__(self):
        super().__init__('imu_fusion')

        # IMU 데이터 구독 (각각의 구독자를 별도로 설정)
        self.accel_sub = self.create_subscription(Calculation, '/calculation/accel', self.accel_callback, 10)
        self.gyro_sub = self.create_subscription(Calculation, '/calculation/gyro', self.gyro_callback, 10)
        self.mag_sub = self.create_subscription(Calculation, '/calculation/mag', self.mag_callback, 10)

        # 보정된 Roll, Pitch, Yaw 발행
        self.orientation_pub = self.create_publisher(Calculation, '/calculation/final', 10)

        # Kalman Filter 설정 (상태 벡터: roll, pitch, yaw)
        self.kf = KalmanFilter(dim_x=3, dim_z=3)
        self.kf.x = np.zeros(3)  # 초기 상태 (roll, pitch, yaw)
        self.kf.F = np.eye(3)  # 상태 전이 행렬
        self.kf.H = np.eye(3)  # 측정 행렬
        self.kf.P *= 1.0  # 초기 오차 공분산 행렬
        self.kf.R *= 0.5  # 측정 노이즈 공분산 행렬
        self.kf.Q *= 0.01  # 프로세스 노이즈 공분산 행렬

        self.get_logger().info("IMU Fusion Node Started!")

    def accel_callback(self, msg):
        # 가속도 기반 Roll, Pitch 계산
        accel_x, accel_y, accel_z = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        self.roll_accel = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2))
        self.pitch_accel = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))

    def gyro_callback(self, msg):
        # 자이로 적분을 통해 Roll, Pitch, Yaw 계산
        gyro_x, gyro_y, gyro_z = msg.angular_acceleration.x, msg.angular_acceleration.y, msg.angular_acceleration.z
        self.dt = 0.02  # 50Hz 가정
        self.roll_gyro = gyro_x * self.dt
        self.pitch_gyro = gyro_y * self.dt
        self.yaw_gyro = gyro_z * self.dt
    
    def mag_callback(self, msg):
        # 자기 센서 기반 Yaw 계산
        mag_x, mag_y = msg.magnetic.yaw, msg.magnetic.pitch
        self.yaw_mag = np.arctan2(mag_y, mag_x)

        # 칼만 필터에 의한 보정
        measured_state = np.array([math.degrees(self.roll_accel), math.degrees(self.pitch_accel), math.degrees(self.yaw_mag)])
        self.kf.predict()
        self.kf.update(measured_state)
        roll, pitch, yaw = self.kf.x

        # 쿼터니언 변환
        qx, qy, qz, qw = self.euler_to_quaternion(math.radians(roll), math.radians(pitch), math.radians(yaw))

        # 보정된 Roll, Pitch, Yaw 발행
        orientation_msg = Calculation()
        orientation_msg.accel.roll = roll
        orientation_msg.accel.pitch = pitch
        orientation_msg.accel.yaw = yaw
        orientation_msg.quaternion.x = qx
        orientation_msg.quaternion.y = qy
        orientation_msg.quaternion.z = qz
        orientation_msg.quaternion.w = qw

        self.orientation_pub.publish(orientation_msg)
        self.get_logger().info(f"IMU 데이터: Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°")

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw
'''

'''
import rclpy
from rclpy.node import Node
from myimu.msg import Calculation
from geometry_msgs.msg import Quaternion
import numpy as np
import math
from filterpy.kalman import KalmanFilter  # pip install filterpy

class IMUFusion(Node):
    def __init__(self):
        super().__init__('imu_fusion')

        # IMU 데이터 구독 (각 센서를 구독)
        self.accel_sub = self.create_subscription(Calculation, '/calculation/accel', self.accel_callback, 10)
        self.gyro_sub = self.create_subscription(Calculation, '/calculation/gyro', self.gyro_callback, 10)
        self.mag_sub = self.create_subscription(Calculation, '/calculation/mag', self.mag_callback, 10)

        # 보정된 Roll, Pitch, Yaw 발행
        self.orientation_pub = self.create_publisher(Calculation, '/calculation/final', 10)

        # Kalman Filter 설정 (상태 벡터: roll, pitch, yaw)
        self.kf = KalmanFilter(dim_x=3, dim_z=3)
        self.kf.x = np.zeros(3)  # 초기 상태 (roll, pitch, yaw)
        self.kf.F = np.eye(3)  # 상태 전이 행렬
        self.kf.H = np.eye(3)  # 측정 행렬
        self.kf.P *= 1.0  # 초기 오차 공분산 행렬
        self.kf.R *= 0.5  # 측정 노이즈 공분산 행렬
        self.kf.Q *= 0.01  # 프로세스 노이즈 공분산 행렬

        self.get_logger().info("✅ IMU Fusion Node Started!")

    def accel_callback(self, msg):
        """가속도 센서를 이용하여 Roll, Pitch 계산"""
        accel_x, accel_y, accel_z = msg.accel.roll, msg.accel.pitch, msg.accel.yaw  # YawPitchRoll 구조체에서 가져오기
        self.roll_accel = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2))
        self.pitch_accel = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))

    def gyro_callback(self, msg):
        """자이로스코프 데이터를 이용하여 Roll, Pitch, Yaw 적분"""
        gyro_x, gyro_y, gyro_z = msg.gyro.roll, msg.gyro.pitch, msg.gyro.yaw  # YawPitchRoll 구조체에서 가져오기
        self.dt = 0.02  # 50Hz 가정
        self.roll_gyro = gyro_x * self.dt
        self.pitch_gyro = gyro_y * self.dt
        self.yaw_gyro = gyro_z * self.dt  # 자이로는 yaw 변화량을 제공

    def mag_callback(self, msg):
        """자기장 센서를 이용하여 Yaw 계산"""
        mag_x, mag_y, mag_z = msg.magnetic.roll, msg.magnetic.pitch, msg.magnetic.yaw  # YawPitchRoll 구조체에서 가져오기
        self.yaw_mag = math.atan2(mag_y, mag_x)  # 자기 센서를 사용한 yaw 계산

        # 칼만 필터에 의한 보정
        measured_state = np.array([
            math.degrees(self.roll_accel), 
            math.degrees(self.pitch_accel), 
            math.degrees(self.yaw_mag)
        ])
        self.kf.predict()
        self.kf.update(measured_state)
        roll, pitch, yaw = self.kf.x

        # Euler → Quaternion 변환
        qx, qy, qz, qw = self.euler_to_quaternion(
            math.radians(roll), 
            math.radians(pitch), 
            math.radians(yaw)
        )

        # 보정된 값 발행
        orientation_msg = Calculation()
        orientation_msg.final.roll = roll
        orientation_msg.final.pitch = pitch
        orientation_msg.final.yaw = yaw
        orientation_msg.orientation.x = qx
        orientation_msg.orientation.y = qy
        orientation_msg.orientation.z = qz
        orientation_msg.orientation.w = qw

        self.orientation_pub.publish(orientation_msg)
        self.get_logger().info(f"✅ 보정된 IMU 데이터: Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Roll, Pitch, Yaw 값을 Quaternion으로 변환"""
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

def main():
    rclpy.init()
    node = IMUFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

import rclpy
from rclpy.node import Node
from myimu.msg import Calculation
from geometry_msgs.msg import Quaternion
import numpy as np
import math
from filterpy.kalman import KalmanFilter  # pip install filterpy

class IMUFusion(Node):
    def __init__(self):
        super().__init__('imu_fusion')

        # IMU 데이터 구독 (가속도, 자이로, 자기장 센서)
        self.accel_sub = self.create_subscription(Calculation, '/calculation/accel', self.imu_callback, 10)
        self.gyro_sub = self.create_subscription(Calculation, '/calculation/gyro', self.imu_callback, 10)
        self.mag_sub = self.create_subscription(Calculation, '/calculation/mag', self.imu_callback, 10)

        # 보정된 Roll, Pitch, Yaw 발행
        self.orientation_pub = self.create_publisher(Calculation, '/calculation/final', 10)

        # Kalman Filter 설정 (상태 벡터: roll, pitch, yaw)
        self.kf = KalmanFilter(dim_x=3, dim_z=3)
        self.kf.x = np.zeros(3)  # 초기 상태 (roll, pitch, yaw)
        self.kf.F = np.eye(3)  # 상태 전이 행렬
        self.kf.H = np.eye(3)  # 측정 행렬
        self.kf.P *= 1.0  # 초기 오차 공분산 행렬
        self.kf.R *= 0.5  # 측정 노이즈 공분산 행렬
        self.kf.Q *= 0.01  # 프로세스 노이즈 공분산 행렬

        self.get_logger().info("✅ IMU Fusion Node Started!")

    def imu_callback(self, msg):
        """가속도, 자이로, 자기장 데이터를 처리하여 Roll, Pitch, Yaw 계산 후 보정"""
        
        # 가속도 센서 데이터 (Roll, Pitch 계산)
        accel_x, accel_y, accel_z = msg.accel.roll, msg.accel.pitch, msg.accel.yaw
        roll_accel = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2))
        pitch_accel = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))

        # 자이로스코프 데이터 (Roll, Pitch, Yaw 적분)
        gyro_x, gyro_y, gyro_z = msg.gyro.roll, msg.gyro.pitch, msg.gyro.yaw
        dt = 0.02  # 50Hz 가정
        roll_gyro = gyro_x * dt
        pitch_gyro = gyro_y * dt
        yaw_gyro = gyro_z * dt  # 자이로는 yaw 변화량을 제공

        # 자기장 센서 데이터 (Yaw 계산)
        mag_x, mag_y, mag_z = msg.magnetic.roll, msg.magnetic.pitch, msg.magnetic.yaw
        yaw_mag = math.atan2(mag_y, mag_x)  # 자기 센서를 사용한 yaw 계산

        # 칼만 필터에 의한 보정
        measured_state = np.array([
            math.degrees(roll_accel + roll_gyro), 
            math.degrees(pitch_accel + pitch_gyro), 
            math.degrees(yaw_mag + yaw_gyro)
        ])
        self.kf.predict()
        self.kf.update(measured_state)
        roll, pitch, yaw = self.kf.x

        # Euler → Quaternion 변환
        qx, qy, qz, qw = self.euler_to_quaternion(
            math.radians(roll), 
            math.radians(pitch), 
            math.radians(yaw)
        )

        # 보정된 값 발행
        orientation_msg = Calculation()
        orientation_msg.final.roll = roll
        orientation_msg.final.pitch = pitch
        orientation_msg.final.yaw = yaw
        orientation_msg.orientation.x = qx
        orientation_msg.orientation.y = qy
        orientation_msg.orientation.z = qz
        orientation_msg.orientation.w = qw

        self.orientation_pub.publish(orientation_msg)
        self.get_logger().info(f"✅ 보정된 IMU 데이터: Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Roll, Pitch, Yaw 값을 Quaternion으로 변환"""
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

def main():
    rclpy.init()
    node = IMUFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
