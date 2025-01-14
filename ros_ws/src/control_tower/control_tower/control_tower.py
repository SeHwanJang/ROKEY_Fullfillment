import sys, queue, smtplib, rclpy, serial, serial.tools.list_ports, keyboard, getkey
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLineEdit, QPushButton, QLabel, QDialog
from PyQt5.uic import loadUi
from type.srv import StartTime
import os
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QTimer
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from std_msgs.msg import Int32

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
        self.password_input.setEchoMode(QLineEdit.Password)  # 비밀번호는 숨김 처리

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

        # 간단한 로그인 인증 (사용자명: 'user', 비밀번호: 'password' 예시)
        if username == "1" and password == "1":
            self.accept()  # 로그인 성공
        else:
            self.reject()  # 로그인 실패


class MainWindow(QMainWindow):
    def __init__(self, ros2_node):
        super().__init__()
        loadUi('/home/bok/control_tower.ui', self)

        # 작업 시작 시간
        self.work_time = 0        
        # ros2 node 사용
        self.ros2_node = ros2_node
        self.bot1 = self.findChild(QLabel,'bot1')
        self.bot2 = self.findChild(QLabel,'bot2')
        self.update_bot1_image('/home/bok/capture_img/capture.jpg')
        self.update_bot2_image('/home/bok/grip_img/grip.jpg')
        # 작업 시간 타이머 설정
        self.work_time = 0
        self.work_time_min = 0
        self.work_time_hour = 0
        self.image_timer = QTimer(self)
        self.timer = QTimer(self)
        self.image_timer.start(100)
        
        # 작업 상태 설정
        self.work_status_text = "대기 중"
        self.work_status.setPlainText(f"작업 상태 : {self.work_status_text}")
        
        self.job1.clicked.connect(self.display_job1)
        self.job2.clicked.connect(self.display_job2)
        self.job3.clicked.connect(self.display_job3)
        
        # 시작 버튼 클릭 이벤트 연결
        self.start.clicked.connect(self.send_message_to_service)
        self.start.clicked.connect(self.on_start_button_click)
        # 정지 버튼 클릭 이벤트 연결
        self.stop.clicked.connect(self.on_stop_button_click)
        # 일시정지 버튼 클릭 이벤트 연결
        self.pause.clicked.connect(self.on_pause_button_click)
        # 계속 버튼 클릭 이벤트
        self.resume.clicked.connect(self.on_resume_button_click)      
        # 리셋 버튼 클릭 이벤트
        self.reset.clicked.connect(self.on_reset_button_click)

        self.constart.clicked.connect(self.constart_button_click)
        self.constop.clicked.connect(self.constop_button_click)
        self.data.clicked.connect(self.data_button_click)
        
        self.saved_messages = {
            'job1': "RED: 2, BLUE: 1",
            'job2': "RED: 1, BLUE: 1",
            'job3': "RED: 1, BLUE: 2"
        }

        self.world = self.findChild(QLabel, 'world')   
        self.timer.timeout.connect(self.update_text)
        self.image_timer.timeout.connect(self.update_image_from_queue)
        self.capture_timer = QTimer(self)
        self.capture_timer.timeout.connect(self.update_images)
        self.capture_timer.start(1000)

    def display_job1(self):
        self.job_list.clear()
        self.job_list.append(self.saved_messages['job1'])

    def display_job2(self):
        self.job_list.clear()
        self.job_list.append(self.saved_messages['job2'])

    def display_job3(self):
        self.job_list.clear()
        self.job_list.append(self.saved_messages['job3'])

    def send_message_to_service(self):
        message_to_send = self.job_list.toPlainText()
        self.ros2_node.call_service('job_list', message_to_send)

    def constart_button_click(self):
        con_status = 1
        self.ros2_node.con_start('constart_button',con_status)


    def constop_button_click(self):
        con_status = 0
        self.ros2_node.con_stop('constop_button',con_status)

    def data_button_click(self):
        data_status = 1
        self.ros2_node.data_save('data_button',data_status)
        

    # 시작 버튼 클릭 시 이벤트
    def on_start_button_click(self):
        self.work_timer.setPlainText(f"작업 시작 : {self.work_time}")
        self.timer.start(1000)
        self.work_status_text = "작업 중"
        self.work_status.setPlainText(f"작업 상태 : {self.work_status_text}")
        
    # 정지 버튼 클릭 시 이벤트
    def on_stop_button_click(self):
        self.timer.stop()
        self.work_timer.setPlainText(f"비상 정지 : {self.work_time}")
        self.work_time = 0
        self.ros2_node.stop_publisher.publish(Int32(data=1))
        
        self.work_status_text = "비상 정지"
        self.work_status.setPlainText(f"작업 상태 : {self.work_status_text}")
        self.send_email("", "", "", "긴급정지", "비상")
        
    # 일시 정지 버튼 클릭 시 이벤트
    def on_pause_button_click(self):
        self.timer.stop()
        self.work_timer.setPlainText(f"일시정지 : {self.work_time}")
        
        
        self.work_status_text = "일시 정지"
        self.work_status.setPlainText(f"작업 상태 : {self.work_status_text}")
    
    # 계속 버튼 클릭 시 이벤트
    def on_resume_button_click(self):
        self.work_timer.setPlainText(str(self.work_time))
        self.timer.start(1000)
        
        self.work_status_text = "재가동"
        self.work_status.setPlainText(f"작업 상태 : {self.work_status_text}")
        QTimer.singleShot(1000, self.update_status_work)

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
            print(f"메일 전송 간 오류 {e}")
    
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

    def update_status_work(self):
        self.work_status_text = "작업 중"
        self.work_status.setPlainText(f"작업 상태 : {self.work_status_text}")


    def update_image_from_queue(self):
        if not self.ros2_node.img_queue.empty():
            frame = self.ros2_node.img_queue.get_nowait()
            self.update_image(frame)

    def update_image(self, frame):
        height, width, channel = frame.shape
        bytes_per_line = 3 * width
        qimage = QImage(frame.data, width, height, bytes_per_line, QImage.Format_BGR888)

        pixmap = QPixmap.fromImage(qimage)
        scaled_pixmap = pixmap.scaled(pixmap.width() // 2, pixmap.height() // 2, aspectRatioMode=1)
        self.world.setPixmap(scaled_pixmap)
    
    def update_images(self):
        """주기적으로 이미지를 업데이트"""
        self.update_bot1_image('/home/bok/capture_img/capture.jpg')
        self.update_bot2_image('/home/bok/grip_img/grip.jpg')

    def update_bot1_image(self, image_path):
        """bot1에 이미지를 QLabel에 표시"""
        if os.path.exists(image_path):  # 이미지 파일이 존재하는지 확인
            try:
                # 이미지 파일이 존재하면 읽어옴
                pixmap = QPixmap(image_path)
                if not pixmap.isNull():
                    # 이미지가 정상적이면 QLabel에 설정
                    self.bot1.setPixmap(pixmap.scaled(self.bot1.size(), aspectRatioMode=1))
                else:
                    print(f"Failed to load image for bot1 from {image_path}")
            except Exception as e:
                print(f"Error loading image for bot1: {e}")
        else:
            pass  # 이미지 파일이 없으면 아무 작업도 하지 않음

    def update_bot2_image(self, image_path):
        """bot2에 이미지를 QLabel에 표시"""
        if os.path.exists(image_path):  # 이미지 파일이 존재하는지 확인
            try:
                # 이미지 파일이 존재하면 읽어옴
                pixmap = QPixmap(image_path)
                if not pixmap.isNull():
                    # 이미지가 정상적이면 QLabel에 설정
                    self.bot2.setPixmap(pixmap.scaled(self.bot2.size(), aspectRatioMode=1))
                else:
                    print(f"Failed to load image for bot2 from {image_path}")
            except Exception as e:
                print(f"Error loading image for bot2: {e}")
        else:
            pass  # 이미지 파일이 없으면 아무 작업도 하지 않음

    def closeEvent(self, event):
        self.ros2_node.stop()
        event.accept()


class ControlTower(Node):
    def __init__(self):
        super().__init__('control_tower_node')
        self.client = self.create_client(StartTime, 'job_list')
        self.constart_client = self.create_client(StartTime, 'constart_button')
        self.constop_client = self.create_client(StartTime, 'constop_button')
        self.data_client = self.create_client(StartTime, 'data_button')

        self.bridge = CvBridge()
        self.img_queue = queue.Queue(maxsize=10)

        self.create_subscription(
            CompressedImage,
            'up_camera',
            self.image_callback,
            10
        )
        self.stop_publisher = self.create_publisher(
            Int32,
            'stop',
            10
        )

    def image_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.img_queue.full():
            self.img_queue.get()
        self.img_queue.put(frame)

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

    def con_stop(self,service_name,con_status):
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

    def data_save(self,service_name,con_status):
        while not self.data_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service {service_name} not available, waiting...")
        request = StartTime.Request()
        request.datastatus = con_status
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

    # QApplication을 먼저 생성
    app = QApplication(sys.argv)
    ros2_node = ControlTower()

    # 로그인 다이얼로그 표시
    login_dialog = LoginDialog()
    if login_dialog.exec_() == QDialog.Accepted:
        # 로그인 성공
        window = MainWindow(ros2_node)
        window.show()

        def ros_spin():
            while rclpy.ok():
                rclpy.spin_once(ros2_node, timeout_sec=0.1)
                app.processEvents()  # PyQt5 이벤트 처리

        ros_spin()

        ros2_node.stop()
        rclpy.shutdown()
    else:
        print("로그인 실패!")

if __name__ == '__main__':
    main()
