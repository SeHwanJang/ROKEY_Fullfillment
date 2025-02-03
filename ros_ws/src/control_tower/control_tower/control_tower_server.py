import os, serial, serial.tools.list_ports, smtplib, getkey, re, cv2, rclpy
import numpy as np
from rclpy.node import Node
from rclpy.timer import Timer

from std_msgs.msg import Int32,Bool
from sensor_msgs.msg import CompressedImage
from type.srv import StartTime

from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart


class ControlTowerServer(Node):
    def __init__(self):
        super().__init__('control_tower_server')
        
        # 작업 목록 전달 서비스
        self.create_service(
            StartTime,
            'job_list',
            self.job_service_callback
        )
        
        # 작업 시작 퍼블리셔
        self.cycle_start_publisher = self.create_publisher(
            Int32,
            'start',
            10
        )          
        
        # RED 작업물 개수 전달 퍼블리셔
        self.red_publisher = self.create_publisher(
            Int32,
            'red_total',
            10
        )
        
        # BLUE 작업물 개수 전달 퍼블리셔
        self.blue_publisher = self.create_publisher(
            Int32,
            'blue_total',
            10
        )   
        
        # 로봇뷰 이미지 서브스크라이버
        self.image_subscriber = self.create_subscription(
            CompressedImage, 
            'turtle_camera', 
            self.image_callback, 
            10
        )
        
        # 컨베이어 벨트 수동 시작 서비스
        self.constart_srv = self.create_service(
            StartTime,
            'constart_button',
            self.handle_constart_button
        )
        
        # 컨베이어 벨트 수동 정지 서비스
        self.constop_srv = self.create_service(
            StartTime,
            'constop_button',
            self.handle_constop_button
        )
        
        # 로봇뷰 이미지 캡쳐 서비스
        self.data_srv = self.create_service(
            StartTime,
            'data_button',
            self.handle_data_button
        )
        
        # 로봇뷰 이미지 구분 서브스크라이버
        self.create_subscription(
            Int32,
            'capture',
            self.capture_callback,
            10
        )
        
        # 컨베이어 벨트 이동 서브스크라이버
        self.drop_subscription = self.create_subscription(
            Int32,
            'drop_condition',
            self.drop_check,
            10
        )
        
        # 작업 완료 전달 서브스크라이버
        self.create_subscription(
            Bool,
            'drop_done',
            self.drop_callback,
            10
        )        
        
        # RED 토픽 발행 여부
        self.red_published = False
        
        # BLUE 토픽 발행 여부
        self.blue_published = False
        
        # RED 작업물 개수
        self.red_count = 0
        
        # BLUE 작업물 개수
        self.blue_count = 0        
        
        # 컨베이어 벨트 이동 여부 변수
        self.drop = 0
        
        # 작업 완료 확인 변수 
        self.drop_done = False
        
        # 로봇뷰 이미지 구분 변수
        # -1 대기, 0 학습 데이터 수집, 1 전체 이미지, 2 근접 이미지
        self.capture = -1
        
        # 저장한 데이터 수집 이미지 개수
        self.image_count = 0
        
        # 컨베이어 벨트 연결
        self.arduino = None
        self.arduino_port = '/dev/ttyACM0'
        self.baudrate = 115200
        
        # 컨베이어 벨트 연결 확인 타이머
        self.arduino_timer = self.create_timer(1.0, self.check_arduino)       

    # 작업 목록 전달 서비스 콜백함수
    def job_service_callback(self, request, response):
        # 요청 값 출력
        self.get_logger().info(f"Received job: {request.job_list}")
        
        # 작업 목록 추출
        job_list = request.job_list
        red_match = re.search(r'RED:\s*(\d+)', job_list)
        blue_match = re.search(r'BLUE:\s*(\d+)', job_list)
        
        if red_match:
            self.red_count = int(red_match.group(1))
        else:
            self.red_count = 0
            
        if blue_match:
            self.blue_count = int(blue_match.group(1))
        else:
            self.blue_count = 0
            
        if len(request.job_list) >= 10:
            response.status = "SUCCESS"
            
            # 작업 시작 토픽 발행
            self.cycle_start_publisher.publish(Int32(data=1))
            self.get_logger().info('start topic published')
            
            # RED 작업물 개수 전달 토픽 발행
            if self.red_published == False:
                self.red_publisher.publish(Int32(data=self.red_count))
                self.red_published = True
            
            # BLUE 작업물 개수 전달 토픽 발행
            if self.blue_published == False:
                self.blue_publisher.publish(Int32(data=self.blue_count))
                self.blue_published = True
                self.get_logger().info('data sended')
        else:
            response.status = "FAILURE"
        
        return response

    # 컨베이어 벨트 수동 시작 서비스 콜백함수
    def handle_constart_button(self, request, response):
        if request.constatus == 1:
            request.datastatus = 0
            self.get_logger().info(f"Coveyor handle start: {request.constatus}")
            response.conv = "Conveyor handle started"
            self.arduino_handle(request)
            
        else:
            response.conv = "conv didn't start"
            self.get_logger().info('con_status is not True')
            
        return response

    # 컨베이어 벨트 수동 정지 서비스 콜백함수
    def handle_constop_button(self, request, response):
        if request.constatus == 0:
            request.datastatus = 0
            self.get_logger().info(f"Conveyor handle stop: {request.constatus}")
            response.conv = "Conveyor handle stopped"
            
        else:
            response.conv = "conv didn't stop"
            self.get_logger().info('con_status is not False')
        
        return response

    # 컨베이어 벨트 수동 동작 함수
    def arduino_handle(self, request):
        while(rclpy.ok()):
            key_value = getkey.getkey()
            
            # z 누르는 동안 동작, q 누르면 종료
            if key_value == 'z':
                self.arduino.write('10'.encode()+b'\n')
            elif key_value == 'q':
                request.constatus = 0
                self.get_logger().info(f'stopped : {request.constatus}')
                break

    # 로봇뷰 이미지 구분 콜백함수
    # 1: 전체 이미지, 2: 근접 이미지
    def capture_callback(self, msg):
        self.capture = msg.data

    # 로봇뷰 이미지 캡쳐 서비스 콜백함수
    def handle_data_button(self, request, response):
        if request.datastatus == 1:
            request.constatus = 0
            self.get_logger().info(f"Data status : {request.datastatus}")

            self.capture = 0
            response.data = "SUCCESS"
            
        else:
            response.data = "FAILURE"
        
        return response
        
    # 로봇뷰 이미지 콜백함수
    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # 학습 데이터 이미지 저장
        if self.capture == 0:
            if frame is not None:
                cv2.imshow("Compressed Image", frame)
                key = cv2.waitKey(1) & 0xFF
                
                # 이미지 저장 종료
                if key == ord('x'):
                    cv2.destroyAllWindows()
                    self.get_logger().info("학습 데이터 수집 종료")
                    
                    # 로봇뷰 이미지 구분 변수 초기화
                    self.capture = -1

                # 이미지 저장
                elif key == ord('z'):
                    save_directory = "/home/bok/data_img"
                    if not os.path.exists(save_directory):
                        os.makedirs(save_directory)
                    image_path = os.path.join(save_directory, f"image_{self.image_count}.jpg")
                    
                    # 저장한 이미지 개수
                    self.image_count += 1
                    
                    cv2.imwrite(image_path, frame)
                    self.get_logger().info(f"{image_path}가 저장되었습니다.")
        
        # 로봇뷰 전체 이미지 저장
        elif self.capture == 1:
            capture_directory = "/home/bok/capture_img"
            if not os.path.exists(capture_directory):
                os.makedirs(capture_directory)
            image_path = os.path.join(capture_directory, f"capture.jpg")
            
            cv2.imwrite(image_path, frame)
            self.get_logger().info(f"{image_path}가 저장되었습니다.")
            
            # 로봇뷰 이미지 구분 변수 초기화
            self.capture = -1
            
        # 로봇뷰 근접 이미지 저장
        elif self.capture == 2:
            grip_directory = "/home/bok/grip_img"
            if not os.path.exists(grip_directory):
                os.makedirs(grip_directory)
            image_path = os.path.join(grip_directory, f"grip.jpg")
            
            cv2.imwrite(image_path, frame)
            self.get_logger().info(f"{image_path}가 저장되었습니다.")
            
            # 로봇뷰 이미지 구분 변수 초기화
            self.capture = -1
    
    # 컨베이어 벨트 이동 콜백함수
    def drop_check(self, msg):
        self.drop = msg.data
        
        # 컨베이어 벨트 소폭 이동
        if self.drop_done == False:
            if self.drop == 1:
                self.arduino.write('1000'.encode()+b'\n')
                self.drop = 0
        
        # 컨베이어 벨트 대폭 이동
        elif self.drop_done == True:
            self.arduino.write('10000'.encode()+b'\n')
            self.drop_done = False    
    
    # 작업 완료 전달 콜백함수
    def drop_callback(self, msg):
        self.drop_done = msg.data
        self.get_logger().info(f'drop_done: {self.drop_done}')
        
    # 컨베이어 벨트 연결 확인 함수
    def check_arduino(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        try:
            # 연결이 없거나, 연결이 끊어진 경우
            if self.arduino is None or not self.arduino.isOpen():
                self.get_logger().info("아두이노 연결 시도 중...")
                self.arduino = serial.Serial(self.arduino_port, self.baudrate, timeout=1)
                self.get_logger().info('연결 완료!')

            # 연결 성공
            if self.arduino.isOpen():
                self.connection = 1

            # 연결 실패
            if '/dev/ttyACM0' not in ports:
                self.send_email("", "", "", "아두이노 연결 실패", "아두이노 연결 실패")
                self.get_logger().warn('연결 실패')
                self.arduino = None
                
        except Exception as e:
            self.get_logger().error(f'오류 사항 : {e}')

    # 이메일 전송
    def send_email(self, email_address, email_password, to_email_address, subject, contents):
        num = 0
        if num == 0:
            try:
                msg = MIMEMultipart()
                msg["From"] = email_address
                msg["To"] = to_email_address
                msg["Subject"] = subject
                msg.attach(MIMEText(contents, 'plain'))
                
                server = smtplib.SMTP_SSL('smtp.naver.com', 465)
                server.login(email_address, email_password)
                
                server.sendmail(email_address, to_email_address, msg.as_string())
                server.quit()
                num += 1
            except Exception as e:
                self.get_logger().error(f"메일 전송 간 오류 {e}")
                    
                    
def main(args=None):
    rclpy.init(args=args)
    
    server = ControlTowerServer()
    rclpy.spin(server)
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
