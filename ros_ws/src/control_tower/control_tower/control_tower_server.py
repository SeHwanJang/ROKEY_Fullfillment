# control_tower_server.py
import rclpy
from rclpy.node import Node
from type.srv import StartTime  # StartTime 서비스 정의

class ControlTowerServer(Node):
    def __init__(self):
        super().__init__('control_tower_server')
        self.create_service(StartTime, 'job_list', self.job_service_callback)

        self.red_count = 0
        self.blue_count = 0

    def job_service_callback(self, request, response):
        # 받은 요청 값 출력
        self.get_logger().info(f"Received job: {request.job_list}")
        self.red_count = request.job_list
        
        # 결과 처리 (예시로 start_value가 1일 때 작업 성공, 아니면 실패)
        if len(request.job_list) >= 10:
            response.status = "SUCCESS"  # 성공 상태
        else:
            response.status = "FAILURE"  # 실패 상태
        
        return response
    
    

def main(args=None):
    rclpy.init(args=args)
    server = ControlTowerServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
