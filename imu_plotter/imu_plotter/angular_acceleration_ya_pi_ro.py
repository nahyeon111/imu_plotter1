import rclpy
from rclpy.node import Node
from myimu.msg import ImuSensor, Calculation
import numpy as np

class Angular_Acceleration_ya_pi_ro(Node):
    def __init__(self):
        super().__init__('imu_orientation_calculator')

        # ✅ IMU 데이터 구독
        self.imu_sub = self.create_subscription(ImuSensor, '/imu/sensor', self.imu_callback, 10)

        # ✅ Roll, Pitch, Yaw 발행
        self.orientation_pub = self.create_publisher(Calculation, '/calculation/gyro', 10)

        # ✅ 초기값 설정
        self.prev_time = None
        self.roll = 0.0  # Roll (x축 회전)
        self.pitch = 0.0  # Pitch (y축 회전)
        self.yaw = 0.0  # Yaw (z축 회전)

        self.get_logger().info("✅ IMU Orientation Calculator Node Started!")

    def imu_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9  # ✅ 초 단위 변환

        # 첫 데이터 수신 시 시간 초기화
        if self.prev_time is None:
            self.prev_time = current_time
            return

        # ✅ 자이로스코프 데이터 가져오기 (rad/s)
        gyro_x = msg.angular_acceleration.x
        gyro_y = msg.angular_acceleration.y
        gyro_z = msg.angular_acceleration.z

        # ✅ 시간 변화량 계산
        dt = current_time - self.prev_time
        self.prev_time = current_time

        if dt <= 0 or dt > 0.1:
            dt = 0.02  # ✅ 50Hz 설정 (사실 100Hz로 들어오고 있음)

        # ✅ Roll, Pitch, Yaw 적분 (단순 오일러 적분)
        self.roll += gyro_x * dt
        self.pitch += gyro_y * dt
        self.yaw += gyro_z * dt

        # ✅ roll, Pitch, yaw 값 발행
        orientation_msg = Calculation()
        orientation_msg.gyro.roll = np.degrees(self.roll)  # ✅ 라디안을 도(degree)로 변환
        orientation_msg.gyro.pitch = np.degrees(self.pitch)
        orientation_msg.gyro.yaw = np.degrees(self.yaw)

        self.orientation_pub.publish(orientation_msg)
        '''
        self.get_logger().info(f"📢 IMU Orientation 발행: Roll={orientation_msg.gyro.roll:.2f}°, Pitch={orientation_msg.gyro.pitch:.2f}°, Yaw={orientation_msg.gyro.yaw:.2f}°")
        '''
def main():
    rclpy.init()
    node = Angular_Acceleration_ya_pi_ro()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
