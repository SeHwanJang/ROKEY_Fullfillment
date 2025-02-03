import rclpy, cv2
import numpy as np
from rclpy.node import Node

from std_msgs.msg import String, Int32,Bool
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge
from ultralytics import YOLO


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # RED 작업물 개수 전달 서브스크라이버
        self.subscription_red_total = self.create_subscription(
            Int32,
            'red_total',
            self.get_total_red,
            10
        )
        
        # BLUE 작업물 개수 전달 서브스크라이버
        self.subscription_blue_total = self.create_subscription(
            Int32,
            'blue_total',
            self.get_total_blue,
            10
        )
        
        # 작업한 RED 개수 전달 서브스크라이버
        self.subscription_red = self.create_subscription(
            Int32,
            'red_num',
            self.get_current_red,
            10
        )
        
        # 작업한 BLUE 개수 전달 서브스크라이버
        self.subscription_blue = self.create_subscription(
            Int32,
            'blue_num',
            self.get_current_blue,
            10
        )
        
        # 작업 위치 도착 서브스크라이버
        self.create_subscription(
            Int32,
            'arrive',
            self.arrive_callback,
            10
        )
        
        # 로봇뷰 이미지 퍼블리셔
        self.publisher_image = self.create_publisher(
            CompressedImage,
            'turtle_camera',
            10
        )
        
        # 작업 물체 색, 위치 전달 퍼블리셔
        self.publisher_num = self.create_publisher(
            String,
            'box_condition',
            10
        )
        
        # 작업 완료 전달 퍼블리셔
        self.publisher_drop = self.create_publisher(
            Bool,
            'drop_done',
            10
        )
        
        # YOLO 모델 로드
        self.model = YOLO("best.pt")

        # 카메라 캡처 설정
        self.capture = cv2.VideoCapture(0)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.capture.isOpened():
            self.get_logger().error("Failed to open camera!")
            return
        
        # RED 작업물 총 개수
        self.total_red = 0
        
        # BLUE 작업물 총 개수
        self.total_blue = 0        
        
        # 작업한 RED 개수
        self.current_red = 0
        
        # 작업한 BLUE 개수
        self.current_blue = 0        
        
        # 작업 위치 도착 확인 변수
        self.arrive = 0
        
        self.bridge = CvBridge()
        
        # 작업 물체 색, 위치, 로봇뷰 이미지 퍼블리시 타이머 설정
        self.timer = self.create_timer(0.1, self.publish_image)        

    # RED 작업물 개수 전달 콜백함수
    def get_total_red(self,msg):
        self.total_red = msg.data
        self.get_logger().info(f'총 빨간색: {self.total_red}')
    
    # BLUE 작업물 개수 전달 콜백함수
    def get_total_blue(self,msg):
        self.total_blue = msg.data
        self.get_logger().info(f'총 파란색: {self.total_blue}')

    # 작업 위치 도착 콜백함수
    def arrive_callback(self,msg):
        self.arrive = msg.data

    # 작업 물체 색, 위치, 로봇뷰 이미지 퍼블리시 함수
    def publish_image(self):
        ret, frame = self.capture.read()
        
        if ret:
            frame = cv2.resize(frame, (640, 480))

            # YOLO 추론 및 구역별 물체 감지
            result_frame, detected_info = self.detect_objects_in_zones(frame)

            # 로봇뷰 이미지 퍼블리시
            self.publish_compressed_image(result_frame)

            # 작업 완료 여부 확인
            if self.total_red == self.current_red and self.total_blue == self.current_blue:
                detected_info == 'go_purple'
            
            # 작업 물체 색, 위치 전달
            # 작업 위치 도착 확인
            if self.arrive == 1:
                self.publisher_num.publish(String(data=detected_info))
                
            # 목표 개수 설정 대기
            else:
                detected_info = 'waiting_topic_come'
                self.publisher_num.publish(String(data=detected_info))

    # 구역별 물체 감지 후 해당 구역 번호, 색상 정보 전달 함수
    def detect_objects_in_zones(self, frame):
        # 구역 번호 초기화
        detected_info = "None"       
        
        # 구역 좌표 설정 
        # 1: 왼쪽 아래, 2: 오른쪽 아래, 3: 왼쪽 위, 4: 오른쪽 위
        zones = {
            1: (0, 370, 312, 480),
            2: (312, 370, 640, 480),
            3: (0, 0, 312, 370),
            4: (312, 0, 640, 370)
        }
        
        # 구역 별 확인
        for zone_num in [1, 2, 3, 4]:
            # 목표 개수 설정 대기
            if self.total_red == 0 and self.total_blue == 0:
                self.get_logger().info('Total값을 기다리는중')
                return frame, detected_info
            
            # 작업 완료 확인
            # 작업 완료 전달 퍼블리셔
            if self.current_red >= self.total_red and self.current_blue >= self.total_blue:
                self.total_red = 0
                self.total_blue = 0
                self.publisher_drop.publish(Bool(data=True))
                break
            
            # 구역에 해당하는 부분 추출
            x1, y1, x2, y2 = zones[zone_num]
            zone_frame = frame[y1:y2, x1:x2]

            # YOLO 추론
            results_in_zone = self.model(zone_frame)

            # 물체 감지 확인
            for box in results_in_zone[0].boxes:
                # 감지된 클래스 ID, 신뢰도
                class_id = int(box.cls)
                confidence = box.conf.item()

                # 신뢰도 0.7 초과 RED, BLUE 구역, 색 추출
                label = self.model.names[class_id]
                if label in ["red", "blue"] and confidence > 0.7:
                    # 해당 색이 작업 완료된 경우 다음 구역으로 넘어감
                    if (label == "red" and self.current_red >= self.total_red) or (label == "blue" and self.current_blue >= self.total_blue):
                        continue
                    
                    # 감지 구역, 색 추출
                    detected_info = f"Zone {zone_num} - {label.capitalize()}"

                    # 바운딩 박스 그리기
                    x1_box, y1_box, x2_box, y2_box = map(int, box.xyxy[0])
                    cv2.rectangle(frame, (x1 + x1_box, y1 + y1_box), (x1 + x2_box, y1 + y2_box), (0, 255, 0), 2)
                    cv2.putText(frame, f'{label} ({confidence:.2f})', 
                                (x1 + x1_box, y1 + y1_box - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 이전 구역에서 감지된 물체 있으면 다른 구역 확인하지 않음
            if detected_info != "None":
                break
        
        return frame, detected_info

    # 로봇뷰 이미지 퍼블리시 함수
    def publish_compressed_image(self, frame):
        # 압축된 이미지로 퍼블리시
        compressed_image_msg = CompressedImage()
        compressed_image_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_image_msg.format = 'jpeg'
        
        # 이미지 압축 품질을 50으로 설정
        _, img_encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        compressed_image_msg.data = np.array(img_encoded).tobytes()
        self.publisher_image.publish(compressed_image_msg)

    # 작업한 RED 개수 전달 콜백함수
    def get_current_red(self,msg):
        self.current_red = msg.data
        self.get_logger().info(f'현재 빨간색: {self.current_red}')
    
    # 작업한 BLUE 개수 전달 콜백함수
    def get_current_blue(self,msg):
        self.current_blue = msg.data
        self.get_logger().info(f'현재 파란색: {self.current_blue}')

    # 노드 종료 시 카메라 리소스 해제
    def __del__(self):
        if self.capture.isOpened():
            self.capture.release()


def main(args=None):
    rclpy.init(args=args)
    
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
