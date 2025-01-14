import os
import serial, serial.tools.list_ports, smtplib
import getkey
import re
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Int32,Bool
from sensor_msgs.msg import CompressedImage
from rclpy.timer import Timer
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from type.srv import StartTime  # StartTime 서비스 정의

save_directory = "/home/bok/data_img"

class ControlTowerServer(Node):
    def __init__(self):
        super().__init__('control_tower_server')
        self.image_count = 1
        self.arduino_port = '/dev/ttyACM0'
        self.baudrate = 115200
        self.arduino = None

        # rclpy.Timer를 사용하여 주기적으로 check_arduino 호출
        self.arduino_timer = self.create_timer(1.0, self.check_arduino)  # 1초마다 호출
        self.create_service(StartTime, 'job_list', self.job_service_callback)
        self.constart_srv = self.create_service(StartTime, 'constart_button', self.handle_constart_button)  # 서비스 생성
        self.constop_srv = self.create_service(StartTime, 'constop_button', self.handle_constop_button)
        self.data_srv = self.create_service(StartTime, 'data_button', self.handle_data_button)
        self.red_publisher = self.create_publisher(Int32,'red_total', 10)
        self.blue_publisher = self.create_publisher(Int32,'blue_total',10)
        self.drop_subscription = self.create_subscription(Int32,'drop_condition',self.drop_check,10)
        self.create_subscription(Bool,'drop_done',self.drop_callback,10)
        self.create_subscription(Int32,'capture',self.capture_callback,10)
        self.cycle_start_publisher = self.create_publisher(Int32,'start',10)
        self.drop_done = False
        self.drop = 0
        self.red_published = False
        self.blue_published = False
        self.image_subscriber = None
        self.show_image = False
        self.red_count = 0
        self.blue_count = 0
        self.capture = 0

    def capture_callback(self,msg):
        self.capture = msg.data
    
    def drop_callback(self,msg):
        self.drop_done = msg.data
        self.get_logger().info(f'drop_done: {self.drop_done}')

    def drop_check(self,msg):
        self.drop = msg.data
        if self.drop_done == False:
            if self.drop == 1:
                self.arduino.write('1000'.encode()+b'\n')  # '1000'을 바이트 형식으로 전송    
                self.drop = 0
        elif self.drop_done == True:
            self.arduino.write('10000'.encode()+b'\n')  # '1000'을 바이트 형식으로 전송   
            self.drop_done = False
        

    def check_arduino(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        try:
            if self.arduino is None or not self.arduino.isOpen():
                # 연결이 없거나, 연결이 끊어진 경우
                print("아두이노 연결 시도 중...")
                self.arduino = serial.Serial(self.arduino_port, self.baudrate, timeout=1)
                print('연결 완료!')

            # 연결이 정상인 경우
            if self.arduino.isOpen():
                self.connection = 1

            if '/dev/ttyACM0' not in ports:
                # 연결 실패 처리
                self.send_email("", "", "", "아두이노 연결 실패", "아두이노 연결 실패")
                print('연결 실패')
                self.arduino = None
        except Exception as e:
            print(f'오류 사항 : {e}')

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
                print(f"메일 전송 간 오류 {e}")

    def job_service_callback(self, request, response):
        # 받은 요청 값 출력
        self.get_logger().info(f"Received job: {request.job_list}")
        job_list = request.job_list
        red_match = re.search(r'RED:\s*(\d+)', job_list)
        blue_match = re.search(r'BLUE:\s*(\d+)', job_list)
        # 결과 처리 (예시로 start_value가 1일 때 작업 성공, 아니면 실패)
        if red_match:
            self.red_count = int(red_match.group(1))
        else:
            self.red_count = 0
        if blue_match:
            self.blue_count = int(blue_match.group(1))
        else:
            self.blue_count = 0
        if len(request.job_list) >= 10:
            response.status = "SUCCESS"  # 성공 상태 
            self.cycle_start_publisher.publish(Int32(data=1))
            self.get_logger().info('start topic published')
            if self.red_published == False:
                self.red_publisher.publish(Int32(data=self.red_count))
                self.red_published = True
            
            if self.blue_published == False:
                self.blue_publisher.publish(Int32(data=self.blue_count))
                self.blue_published = True
                print('data sended')
        else:
            response.status = "FAILURE"  # 실패 상태
        
        
        return response

    def handle_constart_button(self, request, response):
        if request.constatus == 1:
            request.datastatus = 0
            self.get_logger().info(f"Coveyor handle start: {request.constatus}")
            response.conv = "Conveyor handle started"
            self.arduino_handle(request)
        else:
            response.conv = "conv didn't start"
            self.get_logger().info('con_status is not True')
        return response  # 전체 response 객체를 반환
    
    def arduino_handle(self,request):
        while(rclpy.ok()):
            key_value = getkey.getkey()
            
            if key_value == 'z':
                self.arduino.write('10'.encode()+b'\n')
            elif key_value == 'q':
                request.constatus = 0
                self.get_logger().info(f'stopped : {request.constatus}')
                break

    def handle_constop_button(self, request, response):
        if request.constatus == 0:
            request.datastatus = 0
            self.get_logger().info(f"Conveyor handle stop: {request.constatus}")
            response.conv = "Conveyor handle stopped"
        else:
            response.conv = "conv didn't stop"
            self.get_logger().info('con_status is not False')
        
        return response  # 전체 response 객체를 반환



    def handle_data_button(self, request, response):
        if request.datastatus == 1:
            request.constatus = 0
            self.get_logger().info(f"Data status : {request.datastatus}")

            if not self.image_subscriber:
                self.image_subscriber = self.create_subscription(
                    CompressedImage, 
                    'turtle_camera', 
                    self.image_callback, 
                    10
                )
                self.show_image = True
                self.get_logger().info("Subscribed to /camera/compressed")

            response.data = "SUCCESS"
        else:
            self.show_image = False
            response.data = "FAILURE"
        
        return response  # 전체 response 객체를 반환

    def image_callback(self, msg):
        if self.show_image:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if self.capture == 1:
                capture_directory = "/home/bok/capture_img"
                if not os.path.exists(capture_directory):
                    os.makedirs(capture_directory)  # 저장 디렉터리 생성
                image_path = os.path.join(capture_directory, f"capture.jpg")  # 저장 경로 설정
                    
            elif self.capture == 2:
                grip_directory = "/home/bok/grip_img"
                if not os.path.exists(grip_directory):
                    os.makedirs(grip_directory)  # 저장 디렉터리 생성
                image_path = os.path.join(grip_directory, f"grip.jpg")  # 저장 경로 설정
                
            if frame is not None:
                cv2.imshow("Compressed Image", frame)
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('x'):  # 'x'를 눌러 이미지를 멈춤
                    self.show_image = False
                    cv2.destroyAllWindows()
                    self.get_logger().info("Stopped showing images.")
                    self.image_subscriber = None

                elif key == ord('z'):  # 'z'를 눌러 이미지를 저장
                    if not os.path.exists(save_directory):
                        os.makedirs(save_directory)  # 저장 디렉터리 생성
                    image_path = os.path.join(save_directory, f"image_{self.image_count}.jpg")
                    self.image_count+=1
                    cv2.imwrite(image_path, frame)
                    self.get_logger().info(f"Image saved: {image_path}")
                    
                    
def main(args=None):
    rclpy.init(args=args)
    server = ControlTowerServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
