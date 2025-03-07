'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
from myimu.msg import Calculation

class IMUVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscriber = self.create_subscription(Calculation, '/calculation/final', self.callback, 10)

        self.get_logger().info("✅ IMU Visualizer Node Started!")

    def callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "imu_link"

        t.transform.rotation.x = msg.quaternion.x
        t.transform.rotation.y = msg.quaternion.y
        t.transform.rotation.z = msg.quaternion.z
        t.transform.rotation.w = msg.quaternion.w

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = IMUVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
'''
import rclpy
from rclpy.node import Node
from myimu.msg import Calculation
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, TransformStamped
import tf2_ros

class IMUVisualizerAndPosePublisher(Node):
    def __init__(self):
        super().__init__('imu_visualizer_and_pose_publisher')

        # TF 브로드캐스터 설정
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # /calculation/final 구독
        self.subscription = self.create_subscription(
            Calculation,
            '/calculation/final',
            self.callback,
            10
        )
        
        # Pose 메시지 퍼블리셔 생성
        self.pose_publisher = self.create_publisher(Pose, '/calculation/final_pose', 10)
        
        # sensor_msgs/msg/Imu 퍼블리셔 생성
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)

        self.get_logger().info("✅ IMU Visualizer and Pose Publisher Node Started!")

    def callback(self, msg):
        # ✅ Pose 메시지 생성 및 값 할당
        pose_msg = Pose()
        pose_msg.position.x = msg.x
        pose_msg.position.y = msg.y
        pose_msg.position.z = msg.z
        pose_msg.orientation = msg.orientation

        # Pose 메시지 퍼블리시
        self.pose_publisher.publish(pose_msg)
        self.get_logger().info('Pose 메시지를 퍼블리시했습니다.')

        # ✅ TransformStamped 메시지 생성 및 값 할당 (TF2)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "imu_link"

        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = msg.z
        t.transform.rotation = msg.orientation

        # 변환 브로드캐스트
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('TF 변환을 브로드캐스트했습니다.')

        # ✅ sensor_msgs/msg/Imu 변환 및 퍼블리시
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        # Quaternion 설정
        imu_msg.orientation = msg.orientation
        
        # 각속도 (Angular Velocity) 설정
        imu_msg.angular_velocity.x = msg.angular_velocity.x
        imu_msg.angular_velocity.y = msg.angular_velocity.y
        imu_msg.angular_velocity.z = msg.angular_velocity.z

        # 가속도 (Linear Acceleration) 설정
        imu_msg.linear_acceleration.x = msg.linear_acceleration.x
        imu_msg.linear_acceleration.y = msg.linear_acceleration.y
        imu_msg.linear_acceleration.z = msg.linear_acceleration.z

        # Imu 메시지 퍼블리시
        self.imu_publisher.publish(imu_msg)
        self.get_logger().info('IMU 메시지를 퍼블리시했습니다.')

def main():
    rclpy.init()
    node = IMUVisualizerAndPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
'''
# RViz2에서 /visualization/pose 토픽을 구독하고, Pose 메시지(orientation)를 시각화

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from myimu.msg import Calculation

class RvizVisualizer(Node):
    def __init__(self):
        super().__init__('rviz_visualizer')
        
        # Roll, Pitch, Yaw 값을 구독하는 구독자
        self.orientation_sub = self.create_subscription(
            Calculation,
            '/calculation/final',
            self.orientation_callback,
            10
        )
        
        # Pose 메시지 발행을 위한 퍼블리셔
        self.pose_pub = self.create_publisher(PoseStamped, '/visualization/pose', 10)
        
        self.get_logger().info("RViz Visualization Node Started!")

    def orientation_callback(self, msg):
        # 이미 쿼터니언으로 발행된 Roll, Pitch, Yaw를 직접 구독
        qx = msg.final_quaternion.x
        qy = msg.final_quaternion.y
        qz = msg.final_quaternion.z
        qw = msg.final_quaternion.w
        
        # Pose 메시지 생성
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"  # RViz에서 고정 프레임으로 설정된 'map'을 기준으로 설정
        
        # 쿼터니언을 Pose로 변환하여 설정
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        
        # Pose 메시지 발행
        self.pose_pub.publish(pose_msg)

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

# RViz2에서 /visualization/pose 토픽을 구독하고, Pose 메시지(orientation)를 시각화

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from myimu.msg import Calculation

class RvizVisualizer(Node):
    def __init__(self):
        super().__init__('rviz_visualizer')
        
        # Roll, Pitch, Yaw 값을 구독하는 구독자
        self.orientation_sub = self.create_subscription(
            Calculation,
            '/calculation/final',
            self.orientation_callback,
            10
        )
        
        # Pose 메시지 발행을 위한 퍼블리셔
        self.pose_pub = self.create_publisher(PoseStamped, '/visualization/pose', 10)
        
        self.get_logger().info("RViz Visualization Node Started!")

    def orientation_callback(self, msg):
        # 쿼터니언으로 발행된 Roll, Pitch, Yaw를 구독
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        # Pose 메시지 생성
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"  # RViz에서 고정 프레임으로 설정된 'map'을 기준으로 설정
        
        # 쿼터니언을 Pose로 변환하여 설정
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        
        # Pose 메시지 발행
        self.pose_pub.publish(pose_msg)

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
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from myimu.msg import Calculation
from geometry_msgs.msg import Point, Quaternion

class RvizVisualizer(Node):
    def __init__(self):
        super().__init__('rviz_visualizer')
        
        # Roll, Pitch, Yaw 값을 구독하는 구독자
        self.orientation_sub = self.create_subscription(
            Calculation,
            '/calculation/final',
            self.orientation_callback,
            10
        )
        
        # Pose 메시지 발행을 위한 퍼블리셔
        self.pose_pub = self.create_publisher(PoseStamped, '/visualization/pose', 10)

        # Marker 메시지 퍼블리셔 생성 (축을 시각화)
        self.marker_pub = self.create_publisher(Marker, '/visualization/marker', 10)
        
        self.get_logger().info("RViz Visualization Node Started!")

    def orientation_callback(self, msg):
        # 쿼터니언으로 발행된 Roll, Pitch, Yaw를 구독
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        # Pose 메시지 생성
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"  # RViz에서 고정 프레임으로 설정된 'map'을 기준으로 설정
        
        # 쿼터니언을 Pose로 변환하여 설정
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        
        # Pose 메시지 발행
        self.pose_pub.publish(pose_msg)
        
        # Marker 생성 (축을 시각화)
        marker_msg = Marker()
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.header.frame_id = "map"  # 고정 프레임 설정
        marker_msg.ns = "sensor_axes"  # Marker namespace
        marker_msg.id = 0
        marker_msg.type = Marker.ARROW  # 축을 나타내는 화살표 타입
        marker_msg.action = Marker.ADD
        marker_msg.pose.position = Point(0.0, 0.0, 0.0)  # 기준 위치 (로봇의 위치에 따라 변경 가능)

        # 쿼터니언 값으로 회전 적용
        marker_msg.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # 축 크기 설정
        marker_msg.scale.x = 0.5  # 축의 길이
        marker_msg.scale.y = 0.1  # 축의 두께
        marker_msg.scale.z = 0.1  # 축의 두께

        # 색상 설정 (R, G, B, A)
        marker_msg.color.r = 1.0  # 빨강색
        marker_msg.color.g = 0.0  # 녹색
        marker_msg.color.b = 0.0  # 파랑색
        marker_msg.color.a = 1.0  # 투명도

        # Marker 발행
        self.marker_pub.publish(marker_msg)

def main():
    rclpy.init()
    node = RvizVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from myimu.msg import Calculation
from geometry_msgs.msg import Point, Quaternion

class RvizVisualizer(Node):
    def __init__(self):
        super().__init__('rviz_visualizer')
        
        # Roll, Pitch, Yaw 값을 구독하는 구독자
        self.orientation_sub = self.create_subscription(
            Calculation,
            '/calculation/final',
            self.orientation_callback,
            10
        )
        
        # Pose 메시지 발행을 위한 퍼블리셔
        self.pose_pub = self.create_publisher(PoseStamped, '/visualization/pose', 10)

        # Marker 메시지 퍼블리셔 생성 (축을 시각화)
        self.marker_pub = self.create_publisher(Marker, '/visualization/marker', 10)
        
        self.get_logger().info("RViz Visualization Node Started!")

    def orientation_callback(self, msg):
        # 쿼터니언으로 발행된 Roll, Pitch, Yaw를 구독
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        # Pose 메시지 생성
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"  # RViz에서 고정 프레임으로 설정된 'map'을 기준으로 설정
        
        # 쿼터니언을 Pose로 변환하여 설정
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        
        # Pose 메시지 발행
        self.pose_pub.publish(pose_msg)
        
        # Marker 생성 (축을 시각화)
        marker_msg = Marker()
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.header.frame_id = "map"  # 고정 프레임 설정
        marker_msg.ns = "sensor_axes"  # Marker namespace
        marker_msg.id = 0
        marker_msg.type = Marker.ARROW  # 축을 나타내는 화살표 타입
        marker_msg.action = Marker.ADD
        marker_msg.pose.position = Point(0.0, 0.0, 0.0)  # 기준 위치 (로봇의 위치에 따라 변경 가능)

        # 쿼터니언 값으로 회전 적용
        marker_msg.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # 축 크기 설정
        marker_msg.scale.x = 0.5  # 축의 길이
        marker_msg.scale.y = 0.1  # 축의 두께
        marker_msg.scale.z = 0.1  # 축의 두께

        # 색상 설정 (R, G, B, A)
        marker_msg.color.r = 1.0  # 빨강색
        marker_msg.color.g = 0.0  # 녹색
        marker_msg.color.b = 0.0  # 파랑색
        marker_msg.color.a = 1.0  # 투명도

        # Marker 발행
        self.marker_pub.publish(marker_msg)

def main():
    rclpy.init()
    node = RvizVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
