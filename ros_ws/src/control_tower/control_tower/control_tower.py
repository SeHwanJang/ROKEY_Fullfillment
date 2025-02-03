import sys, queue, smtplib, rclpy, os
from rclpy.node import Node

from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLineEdit, QPushButton, QLabel, QDialog
from PyQt5.uic import loadUi
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QTimer

from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage
from type.srv import StartTime

from cv_bridge import CvBridge

from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart


class LoginDialog(QDialog):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("로그인")
        self.setGeometry(100, 100, 300, 150)

        layout = QVBoxLayout()
        
        self.username_label = QLabel("사용자명:")
        self.password_label = QLabel("비밀번호:")
        self.username_input = QLineEdit(self)
        self.password_input = QLineEdit(self)
        
        # 비밀번호 숨김 처리
        self.password_input.setEchoMode(QLineEdit.Password)

        self.login_button = QPushButton("로그인")
        self.login_button.clicked.connect(self.handle_login)

        layout.addWidget(self.username_label)
        layout.addWidget(self.username_input)
        layout.addWidget(self.password_label)
        layout.addWidget(self.password_input)
        layout.addWidget(self.login_button)

        self.setLayout(layout)
        
    def handle_login(self):
        username = self.username_input.text()
        password = self.password_input.text()

        # 로그인 인증
        if username == "1" and password == "1":
            self.accept()
        else:
            self.reject()


class MainWindow(QMainWindow):
    def __init__(self, ros2_node):
        super().__init__()
        loadUi('/home/bok/control_tower.ui', self)
              
        # ros2 node 연결
        self.ros2_node = ros2_node
        
        # Job 버튼 클릭 이벤트 연결
        self.job1.clicked.connect(self.display_job1)
        self.job2.clicked.connect(self.display_job2)
        self.job3.clicked.connect(self.display_job3)
        
        # 시작 버튼 클릭 이벤트 연결
        self.start.clicked.connect(self.on_start_button_click)
        # 작업 목록 전달 이벤트 연결
        self.start.clicked.connect(self.send_message_to_service)
        
        # 정지 버튼 클릭 이벤트 연결
        self.stop.clicked.connect(self.on_stop_button_click)
        
        # 일시정지 버튼 클릭 이벤트 연결
        self.pause.clicked.connect(self.on_pause_button_click)
        
        # 재가동 버튼 클릭 이벤트 연결
        self.resume.clicked.connect(self.on_resume_button_click)   
        
        # 리셋 버튼 클릭 이벤트 연결
        self.reset.clicked.connect(self.on_reset_button_click)

        # 컨베이어 벨트 수동 시작 버튼 클릭 이벤트 연결
        self.constart.clicked.connect(self.constart_button_click)
        
        # 컨베이어 벨트 수동 정지 버튼 클릭 이벤트 연결
        self.constop.clicked.connect(self.constop_button_click)
        
        # 학습 데이터 수집 버튼 클릭 이벤트 연결
        self.data.clicked.connect(self.data_button_click)
        
        # 작업별 작업물 개수 사전 저장
        self.saved_messages = {
            'job1': "RED: 2, BLUE: 1",
            'job2': "RED: 1, BLUE: 1",
            'job3': "RED: 1, BLUE: 2"
        }

        # 작업 상태 설정
        self.work_status_text = "대기 중"
        self.work_status.setPlainText(f"작업 상태 : {self.work_status_text}")

        # 작업 소요 시간
        self.work_time = 0

        # 작업 소요 시간 텍스트 타이머 갱신
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_text)

        # 위젯 저장
        self.world = self.findChild(QLabel, 'world')
        self.bot1 = self.findChild(QLabel,'label')
        self.bot2 = self.findChild(QLabel,'label_2')
        
        # 로봇뷰 이미지 설정
        self.update_bot1_image('/home/bok/capture_img/capture.jpg')
        self.update_bot2_image('/home/bok/grip_img/grip.jpg')
        
        # 월드뷰 0.1초 간격 이미지 타이머 갱신
        self.image_timer = QTimer(self)
        self.image_timer.start(100)
        self.image_timer.timeout.connect(self.update_image_from_queue)
        
        # 로봇뷰 1초 간격 이미지 타이머 갱신
        self.capture_timer = QTimer(self)
        self.capture_timer.timeout.connect(self.update_images)
        self.capture_timer.start(1000)

    # Job1 작업 목록 표시
    def display_job1(self):
        self.job_list.clear()
        self.job_list.append(self.saved_messages['job1'])

    # Job2 작업 목록 표시
    def display_job2(self):
        self.job_list.clear()
        self.job_list.append(self.saved_messages['job2'])

    # Job3 작업 목록 표시
    def display_job3(self):
        self.job_list.clear()
        self.job_list.append(self.saved_messages['job3'])

    # 작업 목록 전달
    def send_message_to_service(self):
        message_to_send = self.job_list.toPlainText()
        self.ros2_node.call_service('job_list', message_to_send)

    # 컨베이어 벨트 수동 시작
    def constart_button_click(self):
        con_status = 1
        self.ros2_node.con_start('constart_button', con_status)

    # 컨베이어 벨트 수동 정지
    def constop_button_click(self):
        con_status = 0
        self.ros2_node.con_stop('constop_button', con_status)

    # 로봇뷰 이미지 캡쳐
    def data_button_click(self):
        data_status = 1
        self.ros2_node.data_save('data_button', data_status)

    # 시작 버튼 클릭 이벤트
    def on_start_button_click(self):
        self.work_timer.setPlainText(f"작업 시작 : {self.work_time}")
        self.timer.start(1000)
        self.work_status_text = "작업 중"
        self.work_status.setPlainText(f"작업 상태 : {self.work_status_text}")
        
    # 정지 버튼 클릭 이벤트
    def on_stop_button_click(self):
        self.timer.stop()
        self.work_timer.setPlainText(f"비상 정지 : {self.work_time}")
        self.work_time = 0
        self.work_status_text = "비상 정지"
        self.work_status.setPlainText(f"작업 상태 : {self.work_status_text}")
        
        # 비상 정지 토픽 발행
        self.ros2_node.stop_publisher.publish(Int32(data=1))
        
        # 비상 정지 시 메일 발송
        # 보내는 이메일, 비밀번호, 받는 이메일, 제목, 내용
        self.send_email("", "", "", "긴급정지", "비상")
    
    # 이메일 전송
    def send_email(self, email_address, email_password, to_email_address, subject, contents):
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
            
        except Exception as e:
            self.get_logger().error(f"메일 전송 간 오류 {e}")
    
    # 일시 정지 버튼 클릭 이벤트
    def on_pause_button_click(self):
        self.timer.stop()
        self.work_timer.setPlainText(f"일시정지 : {self.work_time}")
        self.work_status_text = "일시 정지"
        self.work_status.setPlainText(f"작업 상태 : {self.work_status_text}")
    
    # 계속 버튼 클릭 이벤트
    def on_resume_button_click(self):
        self.work_timer.setPlainText(str(self.work_time))
        self.timer.start(1000)
        self.work_status_text = "재가동"
        self.work_status.setPlainText(f"작업 상태 : {self.work_status_text}")
        QTimer.singleShot(1000, self.update_status_work)
    
    # 리셋 버튼 클릭 시 이벤트
    def on_reset_button_click(self):
        self.timer.stop()
        self.work_timer.setPlainText(f"작업 초기화 : {self.work_time}")
        self.work_time = 0
        self.work_status_text = "작업 초기화"
        self.work_status.setPlainText(f"작업 상태 : {self.work_status_text}")
        QTimer.singleShot(1000, self.update_status_wait)
    
    # 작업시간 업데이트
    def update_text(self):
        self.work_time += 1
        self.work_timer.setPlainText(str(self.work_time))

    # 작업상태 업데이트
    def update_status_wait(self):
        self.work_status_text = "대기 중"
        self.work_status.setPlainText(f"작업 상태 : {self.work_status_text}")

    # 작업상태 업데이트
    def update_status_work(self):
        self.work_status_text = "작업 중"
        self.work_status.setPlainText(f"작업 상태 : {self.work_status_text}")

    # 월드뷰 이미지 큐가 비어있지 않으면 업데이트
    def update_image_from_queue(self):
        if not self.ros2_node.img_queue.empty():
            frame = self.ros2_node.img_queue.get_nowait()
            self.update_image(frame)

    # 월드뷰 이미지 업데이트
    def update_image(self, frame):
        height, width, channel = frame.shape
        bytes_per_line = 3 * width
        qimage = QImage(frame.data, width, height, bytes_per_line, QImage.Format_BGR888)

        pixmap = QPixmap.fromImage(qimage)
        scaled_pixmap = pixmap.scaled(pixmap.width() // 2, pixmap.height() // 2, aspectRatioMode=1)
        self.world.setPixmap(scaled_pixmap)
    
    # 로봇뷰 이미지 업데이트
    def update_images(self):
        self.update_bot1_image('/home/bok/capture_img/capture.jpg')
        self.update_bot2_image('/home/bok/grip_img/grip.jpg')

    # 로봇뷰 이미지 업데이트
    def update_bot1_image(self, image_path):
        # 이미지 파일이 존재 확인
        if os.path.exists(image_path):  
            try:
                pixmap = QPixmap(image_path)
                if not pixmap.isNull():
                    self.bot1.setPixmap(pixmap.scaled(self.bot1.size(), aspectRatioMode=1))
                else:
                    self.get_logger().warn(f"Failed to load image for bot1 from {image_path}")
                    
            except Exception as e:
                self.get_logger().error(f"Error loading image for bot1: {e}")
                
        # 이미지 파일이 없으면 아무 작업도 하지 않음
        else:
            pass

    # 로봇 그립뷰 이미지 업데이트
    def update_bot2_image(self, image_path):
        # 이미지 파일이 존재하는지 확인
        if os.path.exists(image_path):
            try:
                pixmap = QPixmap(image_path)
                if not pixmap.isNull():
                    self.bot2.setPixmap(pixmap.scaled(self.bot2.size(), aspectRatioMode=1))
                else:
                    self.get_logger().warn(f"Failed to load image for bot2 from {image_path}")
                    
            except Exception as e:
                self.get_logger().error(f"Error loading image for bot2: {e}")
                
        # 이미지 파일이 없으면 아무 작업도 하지 않음
        else:
            pass

    def closeEvent(self, event):
        self.ros2_node.stop()
        event.accept()


class ControlTower(Node):
    def __init__(self):
        super().__init__('control_tower_node')
        
        # 작업 목록 전달 클라이언트
        self.client = self.create_client(
            StartTime,
            'job_list'
        )
        
        # 컨베이어 벨트 수동 시작 클라이언트
        self.constart_client = self.create_client(
            StartTime, 
            'constart_button'
        )
        
        # 컨베이어 벨트 수동 정지 클라이언트
        self.constop_client = self.create_client(
            StartTime, 
            'constop_button'
        )
        
        # 로봇뷰 이미지 캡쳐 클라이언트
        self.data_client = self.create_client(
            StartTime, 
            'data_button'
        )

        # 월드뷰 캠 서브스크라이버
        self.create_subscription(
            CompressedImage,
            'up_camera',
            self.image_callback,
            10
        )
        
        # 비상정지 퍼블리셔
        self.stop_publisher = self.create_publisher(
            Int32,
            'stop',
            10
        )
        
        self.bridge = CvBridge()
        # 월드뷰 이미지 큐
        self.img_queue = queue.Queue(maxsize=10)

    # 월드뷰 캠 서브스크라이버 콜백함수
    def image_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.img_queue.full():
            self.img_queue.get()
        self.img_queue.put(frame)

    # 작업 목록 전달 서비스
    def call_service(self, service_name, job_list):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service {service_name} not available, waiting...")
            
        request = StartTime.Request()
        request.job_list = job_list
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response:
            self.get_logger().info(f"Service Response: {response.status}")
        else:
            self.get_logger().warn("No response received from service.")
        
        return response
    
    # 컨베이어 벨트 수동 시작 서비스
    def con_start(self, service_name, con_status):
        while not self.constart_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service {service_name} not available, waiting...")

        request = StartTime.Request()
        request.constatus = con_status
        
        future = self.constart_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response:
            self.get_logger().info(f'Conveyor on: {response.conv}')    
        else:
            self.get_logger().warn('No response received from conveyor')

    # 컨베이어 벨트 수동 정지 서비스
    def con_stop(self, service_name, con_status):
        while not self.constop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service {service_name} not available, waiting...")
            
        request = StartTime.Request()
        request.constatus = con_status
        
        future = self.constop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response:
            self.get_logger().info(f'conveyor on : {response.conv}')
        else:
            self.get_logger().warn('no response received from con')
    
    # 로봇뷰 이미지 캡쳐 서비스
    def data_save(self, service_name, data_status):
        while not self.data_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service {service_name} not available, waiting...")
            
        request = StartTime.Request()
        request.datastatus = data_status
        
        future = self.data_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response:
            self.get_logger().info(f'conveyor on : {response.data}')
        else:
            self.get_logger().warn('no response received from con')

    def stop(self):
        self.destroy_node()


def main():
    rclpy.init()

    app = QApplication(sys.argv)
    ros2_node = ControlTower()
    login_dialog = LoginDialog()
    
    if login_dialog.exec_() == QDialog.Accepted:
        # 로그인 성공
        window = MainWindow(ros2_node)
        window.show()

        def ros_spin():
            while rclpy.ok():
                rclpy.spin_once(ros2_node, timeout_sec=0.1)
                # PyQt5 이벤트 처리
                app.processEvents()

        ros_spin()
        ros2_node.stop()
        rclpy.shutdown()
        
    else:
        print("로그인 실패!")


if __name__ == '__main__':
    main()
