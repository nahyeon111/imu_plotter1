import rclpy
from rclpy.node import Node
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
from myimu.msg import ImuSensor
from threading import Thread
import time

class HTTPHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        """ 안드로이드에서 전송된 HTTP POST 요청을 처리 """
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)

        try:
            # JSON 데이터 파싱
            data = json.loads(post_data.decode('utf-8'))

            # JSON 데이터가 딕셔너리라면 내부에서 리스트 찾기
            if isinstance(data, dict):
                key_with_list = next((key for key in data if isinstance(data[key], list)), None)
                if key_with_list:
                    
                    #print(f"🔍 Found key containing list: {key_with_list}")
                    
                    data = data[key_with_list]  # 내부 리스트로 변환
                else:
                    print("⚠️ Warning: No list found in JSON data")
                    self.send_response(400)
                    self.end_headers()
                    self.wfile.write(b'{"status": "error", "message": "No list found in JSON data"}')
                    return

            # 데이터가 리스트 형태인지 확인
            if isinstance(data, list):
                # 센서 데이터 추출
                accel_data, gyro_data, mag_data = self.extract_sensor_data(data)
                self.server.ros_node.publish_imu(accel_data, gyro_data,mag_data)

            else:
                print("⚠️ Warning: Data is not a list")
                self.send_response(400)
                self.end_headers()
                self.wfile.write(b'{"status": "error", "message": "Data is not a list"}')
                return

            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'{"status": "success"}')
        except json.JSONDecodeError as e:
            print(f"⚠️ Error: Invalid JSON format - {e}")
            self.send_response(400)
            self.end_headers()
            self.wfile.write(b'{"status": "error", "message": "Invalid JSON format"}')
        except Exception as e:
            print(f"⚠️ Error processing JSON: {e}")
            self.send_response(400)
            self.end_headers()
            self.wfile.write(b'{"status": "error"}')
    '''
    def inspect_json_structure(self, data):
        """ JSON 데이터의 최상위 키 확인 및 구조 출력 """
        if isinstance(data, dict):
            print("📌 JSON 최상위 키:", list(data.keys()))
        elif isinstance(data, list):
            print("📌 JSON 최상위 구조가 리스트입니다.")
    '''

    def extract_sensor_data(self, sensor_list):
        """ JSON 데이터에서 가속도계와 자이로스코프 데이터를 추출하는 함수 """
        accel_data = {"x": 0.0, "y": 0.0, "z": 0.0}
        gyro_data = {"x": 0.0, "y": 0.0, "z": 0.0}
        mag_data = {"x": 0.0, "y": 0.0, "z": 0.0}

        if isinstance(sensor_list, list):
            for sensor in sensor_list:
                if isinstance(sensor, dict):
                    sensor_name = sensor.get("name", "").lower()  # 소문자 변환
                    sensor_values = sensor.get("values", {})

                    if not isinstance(sensor_values, dict):
                        print(f"⚠️ Warning: sensor_values is not a dictionary: {sensor_values}")
                        continue

                    if "totalacceleration" in sensor_name:
                        accel_data["x"] = sensor_values.get("x", 0.0)
                        accel_data["y"] = sensor_values.get("y", 0.0)
                        accel_data["z"] = sensor_values.get("z", 0.0)

                    elif "gyroscope" in sensor_name:
                        gyro_data["x"] = sensor_values.get("x", 0.0)
                        gyro_data["y"] = sensor_values.get("y", 0.0)
                        gyro_data["z"] = sensor_values.get("z", 0.0)

                    elif "magnetometer" in sensor_name:
                        mag_data["x"] = sensor_values.get("x", 0.0)
                        mag_data["y"] = sensor_values.get("y", 0.0)
                        mag_data["z"] = sensor_values.get("z", 0.0)
            
            #print(f"[DEBUG] Extracted Acceleration Data: {accel_data}")
            #print(f"[DEBUG] Extracted Gyroscope Data: {gyro_data}")
            #print(f"[DEBUG] Extracted Gyroscope Data: {mag_data}")
            
        return accel_data, gyro_data, mag_data

class HTTPtoROS(Node):
    def __init__(self):
        super().__init__('http_sensor_publisher')
        self.publisher = self.create_publisher(ImuSensor, '/imu/sensor', 10)
        self.get_logger().info("IMU & Magnetic Field Publisher Node has started!")

    def publish_imu(self, accel_data, gyro_data, mag_data):
        imu_msg = ImuSensor()

        # ROS2 메시지가 정상적으로 처리되도록 시간 정보 추가
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        try:
            imu_msg.linear_acceleration.x = float(accel_data.get('x', 0.0))
            imu_msg.linear_acceleration.y = float(accel_data.get('y', 0.0))
            imu_msg.linear_acceleration.z = float(accel_data.get('z', 0.0))

            imu_msg.angular_acceleration.x = float(gyro_data.get('x', 0.0))
            imu_msg.angular_acceleration.y = float(gyro_data.get('y', 0.0))
            imu_msg.angular_acceleration.z = float(gyro_data.get('z', 0.0))

            imu_msg.magnetic_field.x = float(mag_data.get('x', 0.0))
            imu_msg.magnetic_field.y = float(mag_data.get('y', 0.0))
            imu_msg.magnetic_field.z = float(mag_data.get('z', 0.0))

        except Exception as e:
            self.get_logger().error(f"Error processing IMU data: {e}")
            return

        self.publisher.publish(imu_msg)



def start_http_server(ros_node, host='0.0.0.0', port=4000):
    """ HTTP 서버 실행 함수 (멀티스레드 실행) """
    server = HTTPServer((host, port), HTTPHandler)
    server.ros_node = ros_node  # ROS 노드 전달
    print(f"HTTP Server running on {host}:{port}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("HTTP Server shutting down...")
        server.shutdown()

def main():
    rclpy.init()
    ros_node = HTTPtoROS()

    # HTTP 서버를 별도 스레드에서 실행
    server_thread = Thread(target=start_http_server, args=(ros_node,))
    server_thread.daemon = True
    server_thread.start()

    rclpy.spin(ros_node)

if __name__ == '__main__':
    main()

'''
import rclpy
from rclpy.node import Node
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
from myimu.msg import ImuSensor
from threading import Thread
import time

class HTTPHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        
        try:
            data = json.loads(post_data.decode('utf-8'))
            sensor_list = next((v for v in data.values() if isinstance(v, list)), None)
            
            if not sensor_list:
                self.send_error(400, 'No valid sensor list in JSON')
                return
            
            accel, gyro, mag = self.extract_sensor_data(sensor_list)
            self.server.ros_node.publish_imu(accel, gyro, mag)
            self.respond(200, {'status': 'success'})
        except json.JSONDecodeError:
            self.send_error(400, 'Invalid JSON format')
        except Exception as e:
            self.send_error(500, f'Error processing request: {e}')

    def extract_sensor_data(self, sensor_list):
        accel, gyro, mag = ({'x': 0.0, 'y': 0.0, 'z': 0.0} for _ in range(3))
        for sensor in sensor_list:
            if not isinstance(sensor, dict):
                continue
            
            values = sensor.get("values", {})
            if not isinstance(values, dict):
                continue
            
            name = sensor.get("name", "").lower()
            if "totalacceleration" in name:
                accel.update(values)
            elif "gyroscope" in name:
                gyro.update(values)
            elif "magnetometer" in name:
                mag.update(values)
        return accel, gyro, mag

    def respond(self, status_code, message):
        self.send_response(status_code)
        self.end_headers()
        self.wfile.write(json.dumps(message).encode('utf-8'))

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher = self.create_publisher(ImuSensor, '/imu/sensor', 10)

    def publish_imu(self, accel, gyro, mag):
        msg = ImuSensor()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        
        for attr, data in zip(['linear_acceleration', 'angular_acceleration', 'magnetic_field'], [accel, gyro, mag]):
            setattr(msg, attr, type('', (), data)())
        
        self.publisher.publish(msg)

def start_http_server(node, host='0.0.0.0', port=5000):
    server = HTTPServer((host, port), HTTPHandler)
    server.ros_node = node
    print(f"HTTP Server running on {host}:{port}")
    server.serve_forever()

def main():
    rclpy.init()
    node = IMUPublisher()
    Thread(target=start_http_server, args=(node,), daemon=True).start()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
'''