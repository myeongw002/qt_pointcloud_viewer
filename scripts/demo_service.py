#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from u2_icd_pkg.srv import InterestObjs
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from vision_msgs.msg import BoundingBox2D
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
        self.get_logger().info('  1: Send obstacle from TUGV at random position')
        self.get_logger().info('  2: Send person from MUGV at random position')
        self.get_logger().info('  3: Send car from SUGV1 at random position')
        self.get_logger().info('  4: Send custom from SUGV2 at random position')
        self.get_logger().info('  5: Send obstacle from SUAV at random position')
        self.get_logger().info('  r: Send random object from random robot at random position')
        self.get_logger().info('  c: Clear all objects')
        self.get_logger().info('  q: Quit')
        
        # 객체 ID 카운터 (0부터 시작)
        self.object_id_counter = 0
        
        # 키보드 입력을 위한 스레드 시작
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()
        
        # 로봇 목록 (위치는 랜덤으로 생성)
        self.robot_names = ['TUGV', 'MUGV', 'SUGV1', 'SUGV2', 'SUAV']
        
        # 물체 클래스 목록
        self.object_classes = ['obstacle', 'person', 'car', 'custom']
        
        # 기본 압축 이미지 데이터 (빈 이미지)
        self.dummy_image = CompressedImage()
        self.dummy_image.header.frame_id = "camera_link"
        self.dummy_image.format = "jpeg"
        self.dummy_image.data = b'\xff\xd8\xff\xe0\x00\x10JFIF'  # 최소한의 JPEG 헤더

    def get_next_object_id(self):
        """다음 객체 ID를 반환하고 카운터 증가"""
        current_id = self.object_id_counter
        self.object_id_counter += 1
        return current_id

    def generate_random_position(self):
        """랜덤 위치 생성"""
        return [
            random.uniform(-15.0, 15.0),  # X: -15m ~ +15m
            random.uniform(-15.0, 15.0),  # Y: -15m ~ +15m
            random.uniform(0.0, 2.0)      # Z: 0m ~ 5m (지면 위)
        ]

    def keyboard_listener(self):
        """키보드 입력을 감지하는 함수"""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            while rclpy.ok():
                key = sys.stdin.read(1)
                
                if key == '1':
                    pos = self.generate_random_position()
                    self.send_single_object('TUGV', 'obstacle', pos)
                elif key == '2':
                    pos = self.generate_random_position()
                    self.send_single_object('MUGV', 'person', pos)
                elif key == '3':
                    pos = self.generate_random_position()
                    self.send_single_object('SUGV1', 'car', pos)
                elif key == '4':
                    pos = self.generate_random_position()
                    self.send_single_object('SUGV2', 'custom', pos)
                elif key == '5':
                    pos = self.generate_random_position()
                    self.send_single_object('SUAV', 'obstacle', pos)
                elif key == 'r' or key == 'R':
                    # 완전 랜덤: 로봇, 객체 클래스, 위치 모두 랜덤
                    robot = random.choice(self.robot_names)
                    obj_class = random.choice(self.object_classes)
                    pos = self.generate_random_position()
                    self.send_single_object(robot, obj_class, pos)
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

    def send_single_object(self, robot_id, obj_class, position):
        """단일 객체를 서비스로 전송 (길이 1 배열)"""
        request = InterestObjs.Request()
        
        # 헤더 설정
        request.header = Header()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = 'map'
        
        # 로봇 정보
        request.robot_id = robot_id
        request.camera_id = 'main_camera'
        
        # 물체 위치 설정 (position 필드에 물체의 실제 위치)
        request.position = Pose()
        request.position.position.x = float(position[0])
        request.position.position.y = float(position[1]) 
        request.position.position.z = float(position[2])
        request.position.orientation.w = 1.0  # 기본 방향
        
        # 타임스탬프
        request.stamp = self.get_clock().now().to_msg()
        
        # 소스 이미지 (더미 데이터)
        request.source_img = self.dummy_image
        request.source_img.header.stamp = request.stamp
        
        # 단일 객체 정보 (길이 1 배열)
        request.obj_class = [obj_class]  # 길이 1 배열
        
        # 객체 ID (0부터 순차 증가, 길이 1 배열)
        obj_id = self.get_next_object_id()
        request.obj_id = [obj_id]  # 길이 1 배열
        
        # 바운딩 박스 (사용하지 않지만 호환성을 위해 빈 박스 추가)
        bbox = BoundingBox2D()
        request.bbox = [bbox]  # 길이 1 배열
        
        # 서비스 호출
        self.get_logger().info(
            f'🎲 Sending {obj_class} (ID: {obj_id}) from {robot_id} at random position ({position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f})'
        )
        
        future = self.client.call_async(request)
        
        # 콜백 설정
        future.add_done_callback(
            lambda f: self.handle_service_response(f, robot_id, obj_class, obj_id)
        )

    def clear_all_objects(self):
        """모든 객체 클리어"""
        request = InterestObjs.Request()
        
        # 헤더 설정
        request.header = Header()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = 'map'
        
        # 특수 로봇 ID로 클리어 요청
        request.robot_id = 'CLEAR_ALL'
        request.camera_id = 'admin'
        request.position = Pose()
        request.position.orientation.w = 1.0
        request.stamp = self.get_clock().now().to_msg()
        request.source_img = self.dummy_image
        
        # 빈 배열들 (아무것도 추가하지 않음)
        request.obj_class = []
        request.obj_id = []
        request.bbox = []
        
        self.get_logger().info('🗑️  Clearing all objects...')
        
        # 클리어 후 카운터 리셋
        self.object_id_counter = 0
        self.get_logger().info('🔄 Object ID counter reset to 0')
        
        future = self.client.call_async(request)
        future.add_done_callback(
            lambda f: self.handle_service_response(f, 'CLEAR_ALL', 'clear', 0)
        )

    def handle_service_response(self, future, robot_id, obj_class, obj_id):
        """서비스 응답 처리"""
        try:
            response = future.result()
            if response.result:
                if obj_class == 'clear':
                    self.get_logger().info('✅ Successfully cleared all objects')
                else:
                    self.get_logger().info(
                        f'✅ Successfully sent {obj_class} (ID: {obj_id}) from {robot_id}'
                    )
            else:
                self.get_logger().error(
                    f'❌ Failed to process object from {robot_id}'
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