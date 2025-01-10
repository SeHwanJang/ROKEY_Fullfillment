import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool  # SetBool 서비스 임포트
import cv2
import os

#import serial

class ButtonServer(Node):
    def __init__(self):
        super().__init__('button_server')  # 노드 이름 설정
        self.constart_srv = self.create_service(SetBool, 'constart_button', self.handle_constart_button)  # 서비스 생성
        self.constop_srv = self.create_service(SetBool, 'constop_button', self.handle_constop_button)
        
        
        #self.ser = serial.Serial('/dev/ttyACM0', 115200)  # 아두이노가 연결된 포트
        
     
        
    def handle_constart_button(self, request, response):
        if request.data:
            response.success = True
            response.message = "컨베이어 벨트가 작동합니다."
            self.get_logger().info("컨베이어 벨트가 작동합니다.")
            
            
        else:
            response.success = False
            response.message = "컨베이어 시작 버튼이 클릭되지 않았습니다."
            self.get_logger().info("컨베이어 시작 버튼이 클릭되지 않았습니다.")
        return response
        
    def handle_constop_button(self, request, response):
        if request.data:
            response.success = True
            response.message = "컨베이어 벨트가 정지합니다."
            self.get_logger().info("컨베이어 벨트가 정지합니다.")
            
           
        else:
            response.success = False
            response.message = "컨베이어 정지 버튼이 클릭되지 않았습니다."
            self.get_logger().info("컨베이어 정지 버튼이 클릭되지 않았습니다.")
        return response
    


def main(args=None):
    rclpy.init(args=args)
    button_server = ButtonServer()  # 서버 객체 생성
    rclpy.spin(button_server)  # 서버 실행 대기
    rclpy.shutdown()

if __name__ == '__main__':
    main()

