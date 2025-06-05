import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from interfaces_pkg.msg import MotionCommand
import cv2
from cv_bridge import CvBridge
import os
import time

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        
        # 이미지 구독
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # 제어 정보 구독
        self.control_sub = self.create_subscription(
            MotionCommand, 
            '/topic_control_signal', 
            self.data_callback, 
            10)
        
        self.bridge = CvBridge()
        self.save_dir = os.path.expanduser('~/ros2_saved_images')
        os.makedirs(self.save_dir, exist_ok=True)
        
        # 최신 제어 정보 저장용 변수 초기화
        self.steering = 0.0
        self.left_speed = 0.0
        self.right_speed = 0.0
        
        self.get_logger().info(f'이미지 저장 경로: {self.save_dir}')

    def data_callback(self, msg):
        # 제어 정보 수신
        self.steering = msg.steering
        self.left_speed = msg.left_speed
        self.right_speed = msg.right_speed
        self.get_logger().info(f'제어 정보 수신 - Steering: {self.steering}, Left Speed: {self.left_speed}, Right Speed: {self.right_speed}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            timestamp = int(time.time() * 1000)
            
            # 파일 이름에 제어 정보 포함
            filename = os.path.join(
                self.save_dir,
                f'image_{timestamp}_s{self.steering:.2f}_l{self.left_speed:.2f}_r{self.right_speed:.2f}.png'
            )
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f'이미지 저장 완료: {filename}')
        except Exception as e:
            self.get_logger().error(f'이미지 변환 또는 저장 오류: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
