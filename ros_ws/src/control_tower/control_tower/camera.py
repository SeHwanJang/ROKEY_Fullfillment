import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import CompressedImage
import numpy as np
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # 카메라 캡처 설정
        self.capture = cv2.VideoCapture(0)  # 카메라 ID 2번으로 캡처 시작
        #self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        #self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        
        if not self.capture.isOpened():
            self.get_logger().error("Failed to open camera!")
            return
        
        # ROS 퍼블리셔 설정
        self.publisher_image = self.create_publisher(
            CompressedImage,
            'up_camera',
            10  # 큐 사이즈
        )

        # CvBridge 설정
        self.bridge = CvBridge()

        # 타이머 설정: 0.1초마다 이미지 퍼블리시
        self.timer = self.create_timer(0.1, self.publish_image)

    def publish_image(self):
        ret, frame = self.capture.read()
        if ret:
            frame = cv2.resize(frame, (640, 480))  # 이미지 크기 조정

            # 압축된 이미지로 퍼블리시
            self.publish_compressed_image(frame)  
            self.get_logger().info('Image published')

    def publish_compressed_image(self, frame):
        """이미지를 압축하여 퍼블리시"""
        compressed_image_msg = CompressedImage()
        compressed_image_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_image_msg.format = 'jpeg'
        
        # 이미지 압축 품질을 50으로 설정
        _, img_encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])  # 50은 압축 품질
        
        compressed_image_msg.data = np.array(img_encoded).tobytes()

        self.publisher_image.publish(compressed_image_msg)

    def __del__(self):
        # 노드 종료 시 카메라 리소스 해제
        if self.capture.isOpened():
            self.capture.release()

def main():
    rclpy.init()  # ROS 2 초기화
    camera_publisher = CameraPublisher()  # 노드 객체 생성

    rclpy.spin(camera_publisher)  # 노드가 종료될 때까지 실행

    camera_publisher.destroy_node()  # 노드 종료
    rclpy.shutdown()  # ROS 2 종료

if __name__ == '__main__':
    main()
