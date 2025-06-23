#!/usr/bin/env python3
# filepath: /home/myungw00/ROS2/Qt_ws/src/qt_pointcloud_viewer/scripts/demo_interest_objects_client.py

import rclpy
from rclpy.node import Node
from u2_icd_pkg.srv import InterestObjs
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from vision_msgs.msg import BoundingBox2D
# from geometry_msgs.msg import Point2D
from sensor_msgs.msg import CompressedImage
import random
import time
import sys
import termios
import tty
import threading

class DemoInterestObjectsClient(Node):
    def __init__(self):
        super().__init__('demo_interest_objects_client')
        
        # 서비스 클라이언트 생성
        self.client = self.create_client(InterestObjs, 'interest_objects_service')
        
        # 서비스 대기
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Interest Objects service not available, waiting...')
        
        self.get_logger().info('Demo Interest Objects Client ready!')
        self.get_logger().info('Press keys to send demo objects:')
        self.get_logger().info('  1: Send 1 obstacle from TUGV')
        self.get_logger().info('  2: Send 2 objects from MUGV')
        self.get_logger().info('  3: Send 3 objects from SUGV1')
        self.get_logger().info('  4: Send random objects from SUGV2')
        self.get_logger().info('  5: Send custom objects from SUAV')
        self.get_logger().info('  c: Clear all objects (send empty request)')
        self.get_logger().info('  q: Quit')
        
        # 키보드 입력을 위한 스레드 시작
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()
        
        # 데모 데이터
        self.robot_positions = {
            'TUGV': [5.0, 2.0, 0.0],
            'MUGV': [-3.0, -2.0, 0.0],
            'SUGV1': [0.0, 5.0, 0.0],
            'SUGV2': [3.0, -3.0, 0.0],
            'SUAV': [0.0, 0.0, 2.0]  # 공중 로봇
        }
        
        self.object_classes = ['obstacle', 'person', 'car', 'custom', 'unknown']
        
        # 기본 압축 이미지 데이터 (빈 이미지)
        self.dummy_image = CompressedImage()
        self.dummy_image.header.frame_id = "camera_link"
        self.dummy_image.format = "jpeg"
        self.dummy_image.data = b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00\xff\xdb\x00C\x00\x08\x06\x06\x07\x06\x05\x08\x07\x07\x07\t\t\x08\n\x0c\x14\r\x0c\x0b\x0b\x0c\x19\x12\x13\x0f\x14\x1d\x1a\x1f\x1e\x1d\x1a\x1c\x1c $.\' ",#\x1c\x1c(7),01444\x1f\'9=82<.342\xff\xc0\x00\x11\x08\x00\x01\x00\x01\x01\x01\x11\x00\x02\x11\x01\x03\x11\x01\xff\xc4\x00\x1f\x00\x00\x01\x05\x01\x01\x01\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x01\x02\x03\x04\x05\x06\x07\x08\t\n\x0b\xff\xc4\x00\xb5\x10\x00\x02\x01\x03\x03\x02\x04\x03\x05\x05\x04\x04\x00\x00\x01}\x01\x02\x03\x00\x04\x11\x05\x12!1A\x06\x13Qa\x07"q\x142\x81\x91\xa1\x08#B\xb1\xc1\x15R\xd1\xf0$3br\x82\t\n\x16\x17\x18\x19\x1a%&\'()*456789:CDEFGHIJSTUVWXYZcdefghijstuvwxyz\x83\x84\x85\x86\x87\x88\x89\x8a\x92\x93\x94\x95\x96\x97\x98\x99\x9a\xa2\xa3\xa4\xa5\xa6\xa7\xa8\xa9\xaa\xb2\xb3\xb4\xb5\xb6\xb7\xb8\xb9\xba\xc2\xc3\xc4\xc5\xc6\xc7\xc8\xc9\xca\xd2\xd3\xd4\xd5\xd6\xd7\xd8\xd9\xda\xe1\xe2\xe3\xe4\xe5\xe6\xe7\xe8\xe9\xea\xf1\xf2\xf3\xf4\xf5\xf6\xf7\xf8\xf9\xfa\xff\xda\x00\x08\x01\x01\x00\x00?\x00\xffd\xff\xd9'

    def keyboard_listener(self):
        """키보드 입력을 감지하는 함수"""
        # 터미널 설정
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            while rclpy.ok():
                key = sys.stdin.read(1)
                
                if key == '1':
                    self.send_demo_objects('TUGV', 1, ['obstacle'])
                elif key == '2':
                    self.send_demo_objects('MUGV', 2, ['obstacle', 'person'])
                elif key == '3':
                    self.send_demo_objects('SUGV1', 3, ['obstacle', 'car', 'person'])
                elif key == '4':
                    # 랜덤 객체들
                    num_objects = random.randint(1, 4)
                    obj_classes = random.choices(self.object_classes, k=num_objects)
                    self.send_demo_objects('SUGV2', num_objects, obj_classes)
                elif key == '5':
                    self.send_demo_objects('SUAV', 2, ['custom', 'custom'])
                elif key == 'c' or key == 'C':
                    self.clear_all_objects()
                elif key == 'q' or key == 'Q':
                    self.get_logger().info('Exiting...')
                    rclpy.shutdown()
                    break
                elif key == '\x03':  # Ctrl+C
                    break
                    
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def send_demo_objects(self, robot_id, num_objects, obj_classes):
        """데모 객체들을 서비스로 전송"""
        request = InterestObjs.Request()
        
        # 헤더 설정
        request.header = Header()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = 'map'
        
        # 로봇 정보
        request.robot_id = robot_id
        request.camera_id = 'main_camera'
        
        # 로봇 위치 설정
        request.position = Pose()
        if robot_id in self.robot_positions:
            pos = self.robot_positions[robot_id]
            request.position.position.x = float(pos[0])
            request.position.position.y = float(pos[1]) 
            request.position.position.z = float(pos[2])
        
        # 방향 (기본값)
        request.position.orientation.w = 1.0
        
        # 타임스탬프
        request.stamp = self.get_clock().now().to_msg()
        
        # 소스 이미지 (더미 데이터)
        request.source_img = self.dummy_image
        request.source_img.header.stamp = request.stamp
        
        # 객체 정보 생성
        for i in range(num_objects):
            # 객체 클래스
            if i < len(obj_classes):
                request.obj_class.append(obj_classes[i])
            else:
                request.obj_class.append('unknown')
            
            # 객체 ID (랜덤)
            obj_id = random.randint(1000, 9999)
            request.obj_id.append(obj_id)
            
            # 바운딩 박스 생성 (랜덤 위치)
            bbox = BoundingBox2D()
            request.bbox.append(bbox)
        
        # 서비스 호출
        self.get_logger().info(f'Sending {num_objects} objects from {robot_id}: {obj_classes}')
        
        future = self.client.call_async(request)
        
        # 콜백 설정
        future.add_done_callback(
            lambda f: self.handle_service_response(f, robot_id, num_objects)
        )

    def clear_all_objects(self):
        """모든 객체 클리어 (빈 요청 전송)"""
        request = InterestObjs.Request()
        
        # 헤더 설정
        request.header = Header()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = 'map'
        
        # 빈 요청 (객체 없음)
        request.robot_id = 'CLEAR_ALL'
        request.camera_id = 'admin'
        request.position = Pose()
        request.position.orientation.w = 1.0
        request.stamp = self.get_clock().now().to_msg()
        request.source_img = self.dummy_image
        
        # 빈 배열들 (아무것도 추가하지 않음)
        
        self.get_logger().info('Clearing all objects...')
        
        future = self.client.call_async(request)
        future.add_done_callback(
            lambda f: self.handle_service_response(f, 'CLEAR_ALL', 0)
        )

    def handle_service_response(self, future, robot_id, num_objects):
        """서비스 응답 처리"""
        try:
            response = future.result()
            if response.result:
                if num_objects > 0:
                    self.get_logger().info(
                        f'✅ Successfully sent {num_objects} objects from {robot_id}'
                    )
                else:
                    self.get_logger().info('✅ Successfully cleared all objects')
            else:
                self.get_logger().error(
                    f'❌ Failed to process objects from {robot_id}'
                )
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        client = DemoInterestObjectsClient()
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()