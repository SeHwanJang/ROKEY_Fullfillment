import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import String, Int32,Bool
from sensor_msgs.msg import CompressedImage
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # YOLO 모델 로드
        self.model = YOLO("best.pt")  # 사용자 정의 YOLO 모델

        # 카메라 캡처 설정
        self.capture = cv2.VideoCapture(0)  # 카메라 ID 0번으로 캡처 시작
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.capture.isOpened():
            self.get_logger().error("Failed to open camera!")
            return

        # ROS 퍼블리셔 설정
        self.publisher_image = self.create_publisher(
            CompressedImage,
            'up_camera',
            10  # 큐 사이즈
        )
        self.publisher_num = self.create_publisher(
            String,
            'box_condition',
            10
        )
        self.publisher_drop = self.create_publisher(
            Bool,
            'drop_done',
            10
        )

        self.subscription_red = self.create_subscription(Int32,'red_num',self.get_current_red,10)
        self.subscription_blue = self.create_subscription(Int32,'blue_num',self.get_current_blue,10)
        self.subscription_red_total = self.create_subscription(Int32,'red_total',self.get_total_red,10)
        self.subscription_blue_total = self.create_subscription(Int32,'blue_total',self.get_total_blue,10)
        
        self.create_subscription(Int32,'arrive',self.arrive_callback,10)

        self.current_red = 0
        self.current_blue = 0
        self.total_red = 0
        self.total_blue = 0

        # CvBridge 설정
        self.bridge = CvBridge()
        self.arrive = 0

        # 타이머 설정: 0.1초마다 이미지 퍼블리시
        self.timer = self.create_timer(0.1, self.publish_image)
    def get_current_red(self,msg):
        self.current_red = msg.data
        self.get_logger().info(f'현재 빨간색: {self.current_red}')
    def get_current_blue(self,msg):
        self.current_blue = msg.data
        self.get_logger().info(f'현재 파란색: {self.current_blue}')
    def get_total_red(self,msg):
        self.total_red = msg.data
        self.get_logger().info(f'총 빨간색: {self.total_red}')
    def get_total_blue(self,msg):
        self.total_blue = msg.data
        self.get_logger().info(f'총 파란색: {self.total_blue}')
    
    def arrive_callback(self,msg):
        self.arrive = msg.data

    def publish_image(self):
        ret, frame = self.capture.read()
        if ret:
            frame = cv2.resize(frame, (640, 480))  # 이미지 크기 조정

            # YOLO 추론 및 구역별 물체 감지
            result_frame, detected_info = self.detect_objects_in_zones(frame)

            # 압축된 이미지로 퍼블리시
            self.publish_compressed_image(result_frame)
            #self.get_logger().info(f'Image published. Detected: {detected_info}')

            # 물체가 감지된 구역과 색상 정보를 퍼블리시
            if self.total_red == self.current_red and self.total_blue == self.current_blue:
                detected_info == 'go_purple'
            if self.arrive == 1:
                self.publisher_num.publish(String(data=detected_info))
            else:
                detected_info = 'waiting_topic_come'
                self.publisher_num.publish(String(data=detected_info))
            #self.get_logger().info(f'published: {detected_info}')

    def detect_objects_in_zones(self, frame):
        """구역별로 물체 감지 후 해당 구역 번호와 색상 정보 반환"""
        # 구역 좌표 설정
        zones = {
            1: (0, 370, 312, 480),  # 왼쪽 아래
            2: (312, 370, 640, 480),  # 오른쪽 아래
            3: (0, 0, 312, 370),  # 왼쪽 위
            4: (312, 0, 640, 370)  # 오른쪽 위
        }

        # YOLO 추론
        results = self.model(frame)

        # 구역 번호 초기화
        detected_info = "None"


        for zone_num in [1, 2, 3, 4]:  # 우선순위대로 1, 2, 3, 4 순으로 확인
            # 각 구역에서 빨간색과 파란색 감지 여부를 확인하는 조건
            if self.total_red == 0 and self.total_blue == 0:
                self.get_logger().info('Total값을 기다리는중')
                return frame, detected_info
            if (self.current_red >= self.total_red and self.current_blue >= self.total_blue):
                self.total_red = 0
                self.total_blue = 0
                self.publisher_drop.publish(Bool(data=True))
                break  # 현재 빨간색과 파란색이 모두 total 값을 초과하면 더 이상 감지하지 않음

            x1, y1, x2, y2 = zones[zone_num]
            # 구역에 해당하는 부분 추출
            zone_frame = frame[y1:y2, x1:x2]

            # YOLO 모델로 추론
            results_in_zone = self.model(zone_frame)

            # red 또는 blue 물체가 감지되었는지 확인
            for box in results_in_zone[0].boxes:
                class_id = int(box.cls)
                confidence = box.conf.item()

                # 물체가 'red' 또는 'blue'일 경우 해당 구역으로 감지
                label = self.model.names[class_id]
                if label in ["red", "blue"] and confidence > 0.7:
                    # 이미 해당 색이 감지된 경우 다음 구역으로 넘어감
                    if (label == "red" and self.current_red >= self.total_red) or (label == "blue" and self.current_blue >= self.total_blue):
                        continue

                    detected_info = f"Zone {zone_num} - {label.capitalize()}"  # 감지된 구역과 색상 정보를 업데이트

                    # 바운딩 박스를 그리기 (물체 감지)
                    x1_box, y1_box, x2_box, y2_box = map(int, box.xyxy[0])
                    cv2.rectangle(frame, (x1 + x1_box, y1 + y1_box), (x1 + x2_box, y1 + y2_box), (0, 255, 0), 2)
                    cv2.putText(frame, f'{label} ({confidence:.2f})', 
                                (x1 + x1_box, y1 + y1_box - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 감지된 물체가 있으면 더 이상 다른 구역을 확인하지 않음
            if detected_info != "None":
                break
        
        return frame, detected_info


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
