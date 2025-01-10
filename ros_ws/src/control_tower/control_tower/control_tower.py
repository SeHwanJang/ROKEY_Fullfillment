import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLineEdit, QPushButton, QLabel, QDialog
from PyQt5.QtWidgets import QDialogButtonBox
from PyQt5.uic import loadUi
from type.srv import StartTime
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QTimer
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import queue

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
        loadUi('/home/rokey/control_tower.ui', self)
        
        # ros2 node 사용
        self.ros2_node = ros2_node
        
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
        # 리셋 버튼 클릭 이벤트 <<임시 이메일 보내기>>
        self.reset.clicked.connect(self.on_reset_button_click)

        self.saved_messages = {
            'job1': "RED: 2, BLUE: 1",
            'job2': "RED: 1, BLUE: 1",
            'job3': "RED: 1, BLUE: 2"
        }

        self.world = self.findChild(QLabel, 'world')   
        self.timer.timeout.connect(self.update_text)
        self.image_timer.timeout.connect(self.update_image_from_queue)
        

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
        
        
        self.work_status_text = "비상 정지"
        self.work_status.setPlainText(f"작업 상태 : {self.work_status_text}")
        self.send_email("sorkaksema@naver.com", "XPLW2BFKH76J", "sorkaksema@naver.com", "긴급정지", "비상")
        
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

    def closeEvent(self, event):
        self.ros2_node.stop()
        event.accept()


class ControlTower(Node):
    def __init__(self):
        super().__init__('control_tower_node')
        self.client = self.create_client(StartTime, 'job_list')
        self.bridge = CvBridge()
        self.img_queue = queue.Queue(maxsize=10)

        self.create_subscription(
            CompressedImage,
            'up_camera',
            self.image_callback,
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
