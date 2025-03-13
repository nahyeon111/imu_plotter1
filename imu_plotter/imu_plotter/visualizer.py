'''
# ver. first
# IMU 데이터를 TF 프레임으로 변환
# rviz2에서 Fixed Frame을 "map"으로 설정, TF 트리에서 /imu_link로 확인 가능
# rviz2->add->by display type->rvix2_default_plugins->tf,axes

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import TransformStamped
from myimu.msg import Calculation, PosVel
import tf2_ros
import transforms3d.quaternions as quat

class RvizVisualizer(Node):
    def __init__(self):
        super().__init__('rviz_visualizer')

        # QoS 설정
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 위치 데이터를 구독하는 구독자
        self.posvel_sub = self.create_subscription(
            PosVel,
            'PosVel/position',  # 위치 메시지 구독
            self.posvel_callback,
            qos_profile
        )


        # Roll, Pitch, Yaw 값을 구독하는 구독자
        self.orientation_sub = self.create_subscription(
            Calculation,
            '/calculation/final',
            self.orientation_callback,
            qos_profile
        )

        # TF broadcaster 생성
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 초기화 
        self.position = [0.0, 0.0, 0.0] # IMU의 위치: (x, y, z)
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # 초기 자세 (단위: quaternion)

        self.get_logger().info("RViz Visualization Node (TF 기반) Started!")

    def posvel_callback(self, msg):
        """위치 데이터를 업데이트하고 TF 발행"""
        self.position = [msg.position.x, msg.position.y, msg.position.z]
        self.publish_tf()
    
    def orientation_callback(self, msg):
        """IMU 데이터를 받아 자세 업데이트 및 TF 발행"""
        self.orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.publish_tf()

    def publish_tf(self):
        """현재 위치 및 자세 정보를 기반으로 TF 발행"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"   # 기준 좌표계
        t.child_frame_id = "imu_link"  # IMU 센서 좌표계

        # 위치 정보 적용
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]

        # 자세 정보 적용
        t.transform.rotation.x = self.orientation[0]
        t.transform.rotation.y = self.orientation[1]
        t.transform.rotation.z = self.orientation[2]
        t.transform.rotation.w = self.orientation[3]

        # TF 정보 발행
        self.tf_broadcaster.sendTransform(t)
        
        #self.get_logger().info(f"Published TF: Position({self.position[0]}, {self.position[1]}, {self.position[2]}) Orientation({self.orientation[0]}, {self.orientation[1]}, {self.orientation[2]}, {self.orientation[3]})")
        
def main():
    rclpy.init()
    node = RvizVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
'''
# final_1.ver
# IMU 데이터를 TF 프레임으로 변환 + 오도메트리(조향 상태 반영)
# rviz2에서 Fixed Frame을 "map"으로 설정, TF 트리에서 /imu_link로 확인 가능
# rviz2->add->by display type->rvix2_default_plugins->tf,axes
# rviz2->add->by topic->odometry

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import TransformStamped, Vector3
from myimu.msg import Calculation, PosVel
from nav_msgs.msg import Odometry
import tf2_ros
import transforms3d.quaternions as quat

class RvizVisualizer(Node):
    def __init__(self):
        super().__init__('rviz_visualizer')

        # QoS 설정
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 위치 데이터를 구독하는 구독자
        self.pos_sub = self.create_subscription(
            PosVel,
            'PosVel/position',  # 위치 메시지 구독
            self.posvel_callback,
            qos_profile
        )

        # 속도 데이터를 구독하는 구독자
        self.vel_sub = self.create_subscription(
            PosVel,
            'PosVel/velocity',  # 속도 메시지 구독
            self.posvel_callback,
            qos_profile
        )


        # Roll, Pitch, Yaw 값을 구독하는 구독자
        self.orientation_sub = self.create_subscription(
            Calculation,
            '/calculation/final',
            self.orientation_callback,
            qos_profile
        )

        # TF broadcaster 생성
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Odometry publisher 생성
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile)

        # 초기화 
        self.position = [0.0, 0.0, 0.0] # IMU의 위치: (x, y, z)
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # 초기 자세 (단위: quaternion)
        self.velocity = [0.0, 0.0, 0.0]  # 속도 (m/s)

        self.get_logger().info("RViz Visualization Node (TF 기반) Started!")

    def posvel_callback(self, msg):
        """위치 데이터를 업데이트하고 TF 발행"""
        self.position = [msg.position.x, msg.position.y, msg.position.z]
        self.velocity = [msg.velocity.x, msg.velocity.y, msg.velocity.z]
        self.publish_tf()
        self.publish_odometry()
    
    def orientation_callback(self, msg):
        """IMU 데이터를 받아 자세 업데이트 및 TF 발행"""
        self.orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.publish_tf()
        self.publish_odometry()

    def publish_tf(self):
        """현재 위치 및 자세 정보를 기반으로 TF 발행"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"   # 기준 좌표계
        t.child_frame_id = "imu_link"  # IMU 센서 좌표계

        # 위치 정보 적용
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]

        # 자세 정보 적용
        t.transform.rotation.x = self.orientation[0]
        t.transform.rotation.y = self.orientation[1]
        t.transform.rotation.z = self.orientation[2]
        t.transform.rotation.w = self.orientation[3]

        # TF 정보 발행
        self.tf_broadcaster.sendTransform(t)
        
        #self.get_logger().info(f"Published TF: Position({self.position[0]}, {self.position[1]}, {self.position[2]}) Orientation({self.orientation[0]}, {self.orientation[1]}, {self.orientation[2]}, {self.orientation[3]})")
        

    def publish_odometry(self):
        """Odometry 메시지 발행"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "imu_link"

        # 위치 정보 적용
        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = self.position[2]

        # 자세 정보 적용
        odom.pose.pose.orientation.x = self.orientation[0]
        odom.pose.pose.orientation.y = self.orientation[1]
        odom.pose.pose.orientation.z = self.orientation[2]
        odom.pose.pose.orientation.w = self.orientation[3]

        # 속도 정보 적용
        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.linear.y = self.velocity[1]
        odom.twist.twist.linear.z = self.velocity[2]

        # 공분산 값 설정
        # 위치 공분산 (6x6 행렬)
        odom.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # x 방향
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,  # y 방향
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,  # z 방향
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,  # x 방향 회전
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,  # y 방향 회전
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1   # z 방향 회전
        ]

        # 속도 공분산 (6x6 행렬)
        odom.twist.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,  # linear.x
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,  # linear.y
            0.0, 0.0, 0.25, 0.0, 0.0, 0.0,  # linear.z
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,  # angular.x
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,  # angular.y
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01   # angular.z
        ]

        # Odometry 정보 발행
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = RvizVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
'''
# final_2.ver
# IMU 데이터를 TF 프레임으로 변환 + 오도메트리(조향 상태 반영) + 동적으로 경로가 변경되는 경우: 곡선, 직선 경로 번환 코드 추가 
# rviz2에서 Fixed Frame을 "map"으로 설정, TF 트리에서 /imu_link로 확인 가능
# rviz2->add->by display type->rvix2_default_plugins->tf,axes
# rviz2->add->by topic->odometry

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import TransformStamped, Vector3
from myimu.msg import Calculation, PosVel
from nav_msgs.msg import Odometry
import tf2_ros
import transforms3d.quaternions as quat

# 경로 구분: 0은 곡선, 1은 직선
class PathPlanner:
    def __init__(self):
        self.path_type = 0  # 곡선에서 시작
        self.radius = 1.0  # 곡선 경로의 반지름
        self.speed = 0.1  # 속도
        self.x_start = 0.0
        self.y_start = 0.0
        self.angle = 0.0  # 초기 각도

    def get_position(self, t):
        if self.path_type == 0:  # 곡선 경로 (원형)
            x = self.x_start + self.radius * math.cos(self.angle + self.speed * t)
            y = self.y_start + self.radius * math.sin(self.angle + self.speed * t)
            return x, y
        elif self.path_type == 1:  # 직선 경로
            x = self.x_start + self.speed * t
            y = self.y_start
            return x, y
    
    def switch_to_straight(self):
        self.path_type = 1  # 직선 경로로 전환

    def switch_to_curve(self):
        self.path_type = 0  # 곡선 경로로 전환

class RvizVisualizer(Node):
    def __init__(self):
        super().__init__('rviz_visualizer')

        # QoS 설정
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 위치 데이터를 구독하는 구독자
        self.pos_sub = self.create_subscription(
            PosVel,
            'PosVel/position',  # 위치 메시지 구독
            self.posvel_callback,
            qos_profile
        )

        # 속도 데이터를 구독하는 구독자
        self.vel_sub = self.create_subscription(
            PosVel,
            'PosVel/velocity',  # 속도 메시지 구독
            self.posvel_callback,
            qos_profile
        )

        # Roll, Pitch, Yaw 값을 구독하는 구독자
        self.orientation_sub = self.create_subscription(
            Calculation,
            '/calculation/final',
            self.orientation_callback,
            qos_profile
        )

        # TF broadcaster 생성
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Odometry publisher 생성
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile)

        # PathPlanner 객체 생성
        self.path_planner = PathPlanner()
        
        # 초기화 
        self.position = [0.0, 0.0, 0.0] # IMU의 위치: (x, y, z)
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # 초기 자세 (단위: quaternion)
        self.velocity = [0.0, 0.0, 0.0]  # 속도 (m/s)

        self.get_logger().info("RViz Visualization Node (TF 기반) Started!")

    def posvel_callback(self, msg):
        """위치 데이터를 업데이트하고 TF 발행"""
        self.position = [msg.position.x, msg.position.y, msg.position.z]
        self.velocity = [msg.velocity.x, msg.velocity.y, msg.velocity.z]
        self.publish_tf()
        self.publish_odometry()
    
    def orientation_callback(self, msg):
        """IMU 데이터를 받아 자세 업데이트 및 TF 발행"""
        self.orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.publish_tf()
        self.publish_odometry()

    def publish_tf(self):
        """현재 위치 및 자세 정보를 기반으로 TF 발행"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"   # 기준 좌표계
        t.child_frame_id = "imu_link"  # IMU 센서 좌표계

        # 위치 정보 적용
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]

        # 자세 정보 적용
        t.transform.rotation.x = self.orientation[0]
        t.transform.rotation.y = self.orientation[1]
        t.transform.rotation.z = self.orientation[2]
        t.transform.rotation.w = self.orientation[3]

        # TF 정보 발행
        self.tf_broadcaster.sendTransform(t)
        
        #self.get_logger().info(f"Published TF: Position({self.position[0]}, {self.position[1]}, {self.position[2]}) Orientation({self.orientation[0]}, {self.orientation[1]}, {self.orientation[2]}, {self.orientation[3]})")
        

    def publish_odometry(self):
        """Odometry 메시지 발행"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "imu_link"

        # 시간 기반으로 위치 계산 (PathPlanner 사용)
        t = self.get_clock().now().seconds_nanoseconds()[0]  # 시간 (초)
        x, y = self.path_planner.get_position(t)
        self.position = [x, y, 0.0]  # Z축은 0으로 설정

        # 위치 정보 적용
        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = self.position[2]

        # 자세 정보 적용
        odom.pose.pose.orientation.x = self.orientation[0]
        odom.pose.pose.orientation.y = self.orientation[1]
        odom.pose.pose.orientation.z = self.orientation[2]
        odom.pose.pose.orientation.w = self.orientation[3]

        # 속도 정보 적용
        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.linear.y = self.velocity[1]
        odom.twist.twist.linear.z = self.velocity[2]

        # 공분산 값 설정
        # 위치 공분산 (6x6 행렬)
        odom.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # x 방향
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,  # y 방향
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,  # z 방향
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,  # x 방향 회전
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,  # y 방향 회전
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1   # z 방향 회전
        ]

        # 속도 공분산 (6x6 행렬)
        odom.twist.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,  # linear.x
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,  # linear.y
            0.0, 0.0, 0.25, 0.0, 0.0, 0.0,  # linear.z
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,  # angular.x
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,  # angular.y
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01   # angular.z
        ]

        # Odometry 정보 발행
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = RvizVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
'''
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import TransformStamped
from myimu.msg import Calculation, PosVel
import tf2_ros
import transforms3d.quaternions as quat

class RvizVisualizer(Node):
    def __init__(self):
        super().__init__('rviz_visualizer')

        # QoS 설정
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 위치 데이터를 구독하는 구독자
        self.posvel_sub = self.create_subscription(
            PosVel,
            'PosVel/position',
            self.posvel_callback,
            qos_profile
        )

        # Roll, Pitch, Yaw 값을 구독하는 구독자
        self.orientation_sub = self.create_subscription(
            Calculation,
            '/calculation/final',
            self.orientation_callback,
            qos_profile
        )

        # TF broadcaster 생성
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Quaternion 메시지 발행자
        self.quaternion_pub = self.create_publisher(Quaternion, '/visualization/quaternion', qos_profile)

        # Calculation 메시지 발행자
        self.calculation_pub = self.create_publisher(Calculation, '/calculation/final', qos_profile)

        # 초기화 
        self.position = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 1.0]

        self.get_logger().info("RViz Visualization Node (TF 기반) Started!")

    def posvel_callback(self, msg):
        """위치 데이터를 업데이트하고 TF 발행"""
        self.position = [msg.position.x, msg.position.y, msg.position.z]
        self.publish_tf()
    
    def orientation_callback(self, msg):
        """IMU 데이터를 받아 자세 업데이트 및 TF 발행"""
        self.orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.publish_tf()

        # 발행된 메시지를 /calculation/final로 전송
        calculation_msg = Calculation()
        calculation_msg.orientation.x = self.orientation[0]
        calculation_msg.orientation.y = self.orientation[1]
        calculation_msg.orientation.z = self.orientation[2]
        calculation_msg.orientation.w = self.orientation[3]
        self.calculation_pub.publish(calculation_msg)

        # Quaternion 메시지를 /visualization/quaternion로 발행
        quaternion_msg = Quaternion()
        quaternion_msg.x = self.orientation[0]
        quaternion_msg.y = self.orientation[1]
        quaternion_msg.z = self.orientation[2]
        quaternion_msg.w = self.orientation[3]
        self.quaternion_pub.publish(quaternion_msg)

    def publish_tf(self):
        """현재 위치 및 자세 정보를 기반으로 TF 발행"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "imu_link"

        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]

        t.transform.rotation.x = self.orientation[0]
        t.transform.rotation.y = self.orientation[1]
        t.transform.rotation.z = self.orientation[2]
        t.transform.rotation.w = self.orientation[3]

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = RvizVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''