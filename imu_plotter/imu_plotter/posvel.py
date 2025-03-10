# 가속도, 자이로, 지자기계에서 뽑은 상태값에서 칼만필터 적용 후 보정된 roll, pitch, yaw 사용
# 계산 처리 과정 간소화, 오차 줄이기 위함
import rclpy
from rclpy.node import Node
from myimu.msg import ImuSensor, PosVel
import numpy as np

# 중력제거 클래스
class GravityRemover:
    def __init__(self):
        self.orientation = np.zeros(3)  # Roll, Pitch, Yaw (초기값)

    def update_orientation(self, gyro, dt, mag_x, mag_y):
        """자이로 및 지자기 데이터를 이용하여 Roll, Pitch, Yaw 업데이트"""
        self.orientation[0] += gyro[0] * dt  # Roll
        self.orientation[1] += gyro[1] * dt  # Pitch
        self.orientation[2] = np.arctan2(mag_y, mag_x)  # Yaw 보정

    def remove_gravity(self, accel):
        """가속도 데이터에서 중력 성분을 제거"""
        roll, pitch, _ = self.orientation
        g = 9.81  # 중력 가속도 (m/s²)

        # 중력 벡터 계산 (센서 기준 좌표계)
        gravity = np.array([
            -g * np.sin(pitch),
            g * np.sin(roll) * np.cos(pitch),
            g * np.cos(roll) * np.cos(pitch)
        ])

        return accel - gravity

#위치 및 속도 계산 클래스
class IMUPositionEstimator(Node):

    def __init__(self):
        super().__init__('imu_position_estimator')

        # ✅ 센서 데이터 구독
        self.imu_sub = self.create_subscription(ImuSensor, '/imu/sensor', self.imu_callback, 10)
        self.mag_sub = self.create_subscription(ImuSensor, '/android/mag', self.mag_callback, 10)

        # ✅ 데이터 발행
        self.non_gravity_pub = self.create_publisher(PosVel, '/imu/non_gravity_acceleration', 10)
        self.velocity_pub = self.create_publisher(PosVel, '/imu/velocity', 10)
        self.position_pub = self.create_publisher(PosVel, '/imu/position', 10)

        # ✅ 초기값 설정
        self.gravity_remover = GravityRemover()  # 중력 제거 객체 생성
        self.prev_time = None
        self.velocity = np.zeros(3)  # 초기 속도
        self.position = np.zeros(3)  # 초기 위치
        self.mag_x = self.mag_y = 0.0  # 지자기 데이터 초기화

        self.get_logger().info("IMU Position Estimator Node Started!")

    def mag_callback(self, msg):
        """지자기 센서 데이터를 받아 저장"""
        self.mag_x, self.mag_y = msg.magnetic_field.x, msg.magnetic_field.y

    def imu_callback(self, msg):
        """IMU 데이터를 받아 중력 제거 및 속도/위치 계산"""

        current_time = self.get_clock().now().nanoseconds / 1e9  # 초 단위 변환

        if self.prev_time is None:
            self.prev_time = current_time
            return
        
        dt = current_time - self.prev_time
        self.prev_time = current_time

        if dt <= 0 or dt > 0.1:
            self.get_logger().warn(f"⚠️ 비정상적인 dt 감지: {dt:.3f}s → 기본값 0.02s로 보정")
            dt = 0.02  # 50Hz 기준

        # ✅ 가속도 및 자이로 데이터 추출
        accel = np.array([msg.linear_acceleration.x,
                          msg.linear_acceleration.y,
                          msg.linear_acceleration.z])

        gyro = np.array([msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z])

        # ✅ 중력 제거
        self.gravity_remover.update_orientation(gyro, dt, self.mag_x, self.mag_y)
        non_gravity_accel = self.gravity_remover.remove_gravity(accel)

        # ✅ 속도 및 위치 적분
        self.velocity += non_gravity_accel * dt
        self.position += self.velocity * dt

        # ✅ 중력 제거된 가속도 발행
        self.publish_vector(self.non_gravity_pub, non_gravity_accel)

        # ✅ 속도 발행
        self.publish_vector(self.velocity_pub, self.velocity)

        # ✅ 위치 발행
        self.publish_vector(self.position_pub, self.position)

        '''
        # ✅ 디버깅 로그 출력
        self.get_logger().info(f"""
📌 Original Acceleration: x={accel[0]:.3f}, y={accel[1]:.3f}, z={accel[2]:.3f}
🔄 Roll: {np.degrees(self.gravity_remover.orientation[0]):.2f}°, 
    Pitch: {np.degrees(self.gravity_remover.orientation[1]):.2f}°, 
    Yaw: {np.degrees(self.gravity_remover.orientation[2]):.2f}°
🌍 Gravity (sensor frame): x={accel[0] - non_gravity_accel[0]:.3f}, 
    y={accel[1] - non_gravity_accel[1]:.3f}, 
    z={accel[2] - non_gravity_accel[2]:.3f}
🚀 Non-Gravity Acceleration: x={non_gravity_accel[0]:.3f}, y={non_gravity_accel[1]:.3f}, z={non_gravity_accel[2]:.3f}
🏃 Velocity: x={self.velocity[0]:.3f}, y={self.velocity[1]:.3f}, z={self.velocity[2]:.3f}
📍 Position: x={self.position[0]:.3f}, y={self.position[1]:.3f}, z={self.position[2]:.3f}
🕒 Time Step: {dt:.3f}s
        """)
        '''

    def publish_vector(self, publisher, vector):
        """Vector3 메시지로 변환하여 발행"""
        msg = PosVel()
        msg.x, msg.y, msg.z = float(vector[0]), float(vector[1]), float(vector[2])
        publisher.publish(msg)


def main():
    rclpy.init()
    node = IMUPositionEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

