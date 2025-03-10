'''
# 가속도, 자이로, 지자기계에서 뽑은 상태값에서 칼만필터 적용 후 보정된 roll, pitch, yaw 사용
# 계산 처리 과정 간소화, 오차 줄이기 위함
import rclpy
from rclpy.node import Node
from myimu.msg import ImuSensor, Calculation, PosVel
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

        '
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
        '

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
'''
'''
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Vector3, Quaternion
from myimu.msg import ImuSensor, PosVel  # 사용자 정의 메시지 가져오기
from tf_transformations import quaternion_multiply, quaternion_inverse # sudo apt install ros-${ROS_DISTRO}-tf-transformations

class IMUVelocityPosition(Node):
    def __init__(self):
        super().__init__('imu_velocity_position')

        # 속도 및 위치 초기화
        self.velocity = np.array([0.0, 0.0, 0.0])  # 속도 (m/s)
        self.position = np.array([0.0, 0.0, 0.0])  # 위치 (m)

        # 마지막 시간 초기화
        self.last_time = None

        # IMU 센서 및 EKF 보정된 쿼터니언 구독
        self.imu_sub = self.create_subscription(ImuSensor, '/imu/sensor', self.imu_callback, 10)
        self.create_subscription(Quaternion, '/calculation/final', self.quaternion_callback, 10)

        # 속도 및 위치 퍼블리셔
        self.posvel_pub_position = self.create_publisher(PosVel, 'PosVel/position', 10)
        self.posvel_pub_velocity = self.create_publisher(PosVel, 'PosVel/velocity', 10)
        self.posvel_pub_non_gravity = self.create_publisher(PosVel, 'PosVel/non_gravity', 10)


        # 보정된 쿼터니언 초기화
        self.corrected_quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # 기본 단위 쿼터니언

    def quaternion_callback(self, msg):
        """ EKF 보정된 쿼터니언 업데이트 """
        self.corrected_quaternion = np.array([msg.w, msg.x, msg.y, msg.z])

    def imu_callback(self, msg):
        """ IMU 데이터를 받아서 속도 및 위치 계산 """
        if self.last_time is None:
            self.last_time = self.get_clock().now()
            return

        # 현재 시간
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # 초 단위 변환
        self.last_time = current_time

        if dt <= 0:
            return

        # 가속도 데이터 추출
        accel_imu = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        # 중력 보정 (쿼터니언을 이용해 가속도 변환)
        accel_world = self.transform_acceleration(accel_imu, self.corrected_quaternion)

        # 중력 제거
        gravity = np.array([0.0, 0.0, 9.81])
        accel_corrected = accel_world - gravity

        # 속도 계산 (적분)
        self.velocity += accel_corrected * dt

        # 위치 계산 (적분)
        self.position += self.velocity * dt

        # 메시지 생성 및 퍼블리시
        posvel_msg = PosVel()
        posvel_msg.header.stamp = current_time.to_msg()
        posvel_msg.header.frame_id = "imu_link"

        # 중력 제거된 가속도를 쿼터니언으로 저장
        posvel_msg.non_gravity.x, posvel_msg.non_gravity.y, posvel_msg.non_gravity.z, posvel_msg.non_gravity.w = (accel_corrected.tolist() + [1.0])

        # 속도 저장
        posvel_msg.velocity.x, posvel_msg.velocity.y, posvel_msg.velocity.z = self.velocity

        # 위치 저장
        posvel_msg.position.x, posvel_msg.position.y, posvel_msg.position.z = self.position

        # 퍼블리시
        self.posvel_pub.publish(posvel_msg)

    def transform_acceleration(self, accel, quaternion):
        """IMU 좌표계에서 월드 좌표계로 가속도 변환"""
        q_conj = quaternion_inverse(quaternion)  # 쿼터니언 역수 계산
        accel_quat = np.array([0.0, accel[0], accel[1], accel[2]])
        transformed_accel = quaternion_multiply(quaternion_multiply(quaternion, accel_quat), q_conj)
        return transformed_accel[1:]  # x, y, z 값 반환

def main(args=None):
    rclpy.init(args=args)
    node = IMUVelocityPosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Vector3, Quaternion
from myimu.msg import ImuSensor, PosVel  # 사용자 정의 메시지 가져오기
from tf_transformations import quaternion_multiply, quaternion_inverse # sudo apt install ros-${ROS_DISTRO}-tf-transformations

class IMUVelocityPosition(Node):
    def __init__(self):
        super().__init__('imu_velocity_position')

        # 속도 및 위치 초기화
        self.velocity = np.array([0.0, 0.0, 0.0])  # 속도 (m/s)
        self.position = np.array([0.0, 0.0, 0.0])  # 위치 (m)

        # 마지막 시간 초기화
        self.last_time = None

        # IMU 센서 및 EKF 보정된 쿼터니언 구독
        self.imu_sub = self.create_subscription(ImuSensor, '/imu/sensor', self.imu_callback, 10)
        self.create_subscription(Quaternion, '/calculation/final', self.quaternion_callback, 10)

        # 각기 다른 퍼블리셔 할당
        self.posvel_pub_position = self.create_publisher(PosVel, 'PosVel/position', 10)
        self.posvel_pub_velocity = self.create_publisher(PosVel, 'PosVel/velocity', 10)
        self.posvel_pub_non_gravity = self.create_publisher(PosVel, 'PosVel/non_gravity', 10)

        # 보정된 쿼터니언 초기화
        self.corrected_quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # 기본 단위 쿼터니언

    def quaternion_callback(self, msg):
        """ EKF 보정된 쿼터니언 업데이트 """
        self.corrected_quaternion = np.array([msg.w, msg.x, msg.y, msg.z])

    def imu_callback(self, msg):
        """ IMU 데이터를 받아서 속도 및 위치 계산 """
        if self.last_time is None:
            self.last_time = self.get_clock().now()
            return

        # 현재 시간
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # 초 단위 변환
        self.last_time = current_time

        if dt <= 0:
            return

        # 가속도 데이터 추출
        accel_imu = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        # 중력 보정 (쿼터니언을 이용해 가속도 변환)
        accel_world = self.transform_acceleration(accel_imu, self.corrected_quaternion)

        # 중력 제거
        gravity = np.array([0.0, 0.0, 9.81])
        accel_corrected = accel_world - gravity

        # 속도 계산 (적분)
        self.velocity += accel_corrected * dt

        # 위치 계산 (적분)
        self.position += self.velocity * dt

        # 메시지 생성
        posvel_msg = PosVel()
        posvel_msg.header.stamp = current_time.to_msg()
        posvel_msg.header.frame_id = "imu_link"

        # 중력 제거된 가속도를 쿼터니언으로 저장
        posvel_msg.non_gravity.x, posvel_msg.non_gravity.y, posvel_msg.non_gravity.z, posvel_msg.non_gravity.w = (accel_corrected.tolist() + [1.0])

        # 속도 저장
        posvel_msg.velocity.x, posvel_msg.velocity.y, posvel_msg.velocity.z = self.velocity

        # 위치 저장
        posvel_msg.position.x, posvel_msg.position.y, posvel_msg.position.z = self.position

        # 퍼블리시
        self.posvel_pub_position.publish(posvel_msg)  # 위치 퍼블리시
        self.posvel_pub_velocity.publish(posvel_msg)  # 속도 퍼블리시
        self.posvel_pub_non_gravity.publish(posvel_msg)  # 중력 제거된 가속도 퍼블리시

    def transform_acceleration(self, accel, quaternion):
        """IMU 좌표계에서 월드 좌표계로 가속도 변환"""
        q_conj = quaternion_inverse(quaternion)  # 쿼터니언 역수 계산
        accel_quat = np.array([0.0, accel[0], accel[1], accel[2]])
        transformed_accel = quaternion_multiply(quaternion_multiply(quaternion, accel_quat), q_conj)
        return transformed_accel[1:]  # x, y, z 값 반환
    
def main(args=None):
    rclpy.init(args=args)
    node = IMUVelocityPosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()