import rclpy, time, math
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Int32, Header, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # 매니퓰레이터 동작 퍼블리셔
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        # 그리퍼 상태 클라이언트
        self.gripper_action_client = ActionClient(self, 
            GripperCommand, 
            'gripper_controller/gripper_cmd'
        )
        
        # 비상정지 서브스크라이버
        self.create_subscription(
            Int32,
            'stop',
            self.stop_callback,
            10
        )
        
        # 작업 위치 도착 서브스크라이버
        self.create_subscription(
            Int32,
            'arrive',
            self.arrive_callback,
            10
        )        
        
        # 작업 물체 위치, 색 전달 서브스크라이버
        self.create_subscription(
            String, 
            'box_condition',
            self.box_condition,
            10
        )

        # 작업한 RED 개수 전달 퍼블리셔
        self.red_count = self.create_publisher(
            Int32,
            'red_num',
            10
        )
        
        # 작업한 BLUE 개수 전달 퍼블리셔
        self.blue_count = self.create_publisher(
            Int32,
            'blue_num',
            10
        )
        
        # 로봇뷰 이미지 구분 퍼블리셔
        self.capture_publisher = self.create_publisher(
            Int32,
            'capture',
            10
        )        
        
        # 컨베이어 벨트 이동 퍼블리셔
        self.drop_pub = self.create_publisher(
            Int32,
            'drop_condition',
            10
        )
        
        # 비상 정지 확인 변수
        self.stop = 0     
           
        # 작업 위치 도착 확인 변수
        self.arrive = 0
        
        # 작업 구역 위치 변수
        self.box_position = 0
        
        # 작업할 물체 색
        self.detected_color = None        
        
        # 작업 중 표시 변수
        self.ignore = False        
        
        # 작업한 RED 개수
        self.red = 0
        
        # 작업한 BLUE 개수
        self.blue = 0        
        
        # 매니퓰레이터 홈포지션 카운터
        self.move_count = 0
        
        # 매니퓰레이터 정보 계산
        # 링크1
        self.r1 = 130
        # 링크2
        self.r2 = 124
        # 링크3
        self.r3 = 126
        # 링크 기울어짐 보정값
        self.th1_offset = - math.atan2(0.024, 0.128)
        # 링크 기울어짐 보정값2 (월드 좌표계 기준이라 90을 뺌, 시계방향 회전이라 마이너스)
        self.th2_offset = - 0.5 * math.pi - self.th1_offset
    
    # 비상정지 콜백함수
    def stop_callback(self, msg):
        self.stop = msg.data
        if self.stop == 1:
            self.destroy_node()
            rclpy.shutdown()

    # 작업 위치 도착 콜백함수
    def arrive_callback(self, msg):
        self.arrive = msg.data
        
        # 작업 테이블 도착
        if self.arrive == 1:
            # 매니퓰레이션 홈포지션 이동
            self.mani_home()
            # 그리퍼 상태 설정
            self.send_gripper_goal(0.025)
            # 매니퓰레이션 동작
            self.move()

        # 작업 상자 위치 도착
        if self.arrive == 2:
            self.get_logger().info('보라박스')
            self.send_gripper_goal(0.025)
            time.sleep(2)
            
            self.move_arm_to_position(0,150,200)
            time.sleep(2)
            
            self.move_arm_to_position(0,190,140)
            time.sleep(2)
            
            self.send_gripper_goal(-0.15)
            time.sleep(2)
            
            # 컨베이어벨트에 걸리지 않게 소폭 이동
            self.trajectory_msg.points[0].positions[0] += 0.2
            self.joint_pub.publish(self.trajectory_msg)
            time.sleep(1)
            
            self.move_arm_to_position(0,40,230)
            time.sleep(1)
            
            self.move_arm_to_position(0,-40,230)
        
        # 목표 지점 도착
        if self.arrive == 3:
            self.get_logger().info('박스 내려놓기')
            self.move_arm_to_position(0,-200,150)
            time.sleep(2)
            
            self.send_gripper_goal(0.025)
            time.sleep(2)
            
            self.move_arm_to_position(0,-100,200)
            time.sleep(2)
            
            self.mani_home()
            time.sleep(2)
            
    # 매니퓰레이션 홈포지션 함수
    def mani_home(self):
        self.trajectory_msg.points[0].positions[0] = 0.0
        self.trajectory_msg.points[0].positions[1] = -0.845402829343338
        self.trajectory_msg.points[0].positions[2] = 0.47597600165170617
        self.trajectory_msg.points[0].positions[3] = 1.4402231544865285            
        self.joint_pub.publish(self.trajectory_msg)

        # 매니퓰레이터 홈포지션 카운터
        self.move_count +=1
        time.sleep(5)
        
    # 그리퍼 상태 설정 함수
    def send_gripper_goal(self, position):
        goal = GripperCommand.Goal()
        # 그리퍼 목표 위치 (0 닫힘 / 1 열림)
        goal.command.position = position
        # 힘 제한 없음
        goal.command.max_effort = -1.0
        self.gripper_action_client.send_goal_async(goal)
        time.sleep(2)
        
    # 작업 구역 위치 업데이트 대기 타이머 설정
    def move(self):
        self.timer = self.create_timer(0.1, self.check_box_position)
        
    # 매니퓰레이터 동작 함수
    def check_box_position(self):
        # 작업할 구역, 작업 중 여부 확인
        if self.box_position != 0 and not self.ignore:
            
            # box_postion 업데이트 대기 타이머 종료
            self.timer.cancel()
            
            # 작업한 RED, BLUE 개수 전달
            if self.detected_color == 'Red':
                self.red += 1
                self.red_count.publish(Int32(data = self.red))
                self.get_logger().info(f'published RED count {self.red}')
            
            elif self.detected_color == 'Blue':
                self.blue += 1
                self.blue_count.publish(Int32(data = self.blue))
                self.get_logger().info(f'published BLUE count {self.blue}')

            # 작업 중 표시
            self.ignore = True
            self.get_logger().info(f'좌표 확인 된 곳: {self.box_position}')
            
            # 로봇뷰 이미지 구분 퍼블리시
            self.capture_publisher.publish(Int32(data=1))
            
            # 작업 실시
            if self.box_position == 1:
                self.grip_one()
            elif self.box_position == 2:
                self.grip_two()
            elif self.box_position == 3:
                self.grip_three()
            elif self.box_position == 4:
                self.grip_four()
            else:
                pass

            # 로봇뷰 이미지 구분 퍼블리시
            self.capture_publisher.publish(Int32(data=2))
            
            # 작업물 들기
            self.trajectory_msg.points[0].positions[1] -= 0.5
            self.joint_pub.publish(self.trajectory_msg)
            time.sleep(2)
            
            # 컨베이어 벨트 위로 옮기기
            self.drop_con()
            self.mani_home()
            
            # 작업 구역, 색 초기화
            self.box_position = 0
            self.detected_color = " "
            self.get_logger().info(f'박스 위치: {self.box_position}    박스 색: {self.detected_color}')
            
            # 작업 구역 위치 업데이트 대기 타이머 설정
            self.timer = self.create_timer(0.1, self.check_box_position)        
    
    # 작업 물체 위치, 색 전달 콜백함수
    def box_condition(self, msg):
        # 작업 중 여부 확인
        if not self.ignore:
            try:
                detected_info = msg.data
                
                if detected_info == 'go_purple':
                    return
                
                else:
                    parts = detected_info.split(" - ")
                    if len(parts) == 2:
                        self.box_position = int(parts[0].split(" ")[1])
                        self.detected_color = parts[1]
                        
            except Exception as e:
                self.get_logger().error(f"Failed to parse detected info: {str(e)}")    
                
    # 1구역 작업
    def grip_one(self):
        self.move_arm_to_position(145, 60, 50)
        time.sleep(2)
        self.get_logger().info('1번 자리 그립')
        self.send_gripper_goal(-0.15)
        
    # 2구역 작업
    def grip_two(self):
        self.move_arm_to_position(145, -70, 50)
        time.sleep(2)
        self.get_logger().info('2번 자리 그립')
        self.send_gripper_goal(-0.15)
        
    # 3구역 작업
    def grip_three(self):
        self.move_arm_to_position(200, 65, 50)
        time.sleep(2)
        self.get_logger().info('3번 자리 그립')
        self.send_gripper_goal(-0.15)
    
    # 4구역 작업
    def grip_four(self):
        self.move_arm_to_position(200, -70, 50)
        time.sleep(2)
        self.get_logger().info('4번 자리 그립')
        self.send_gripper_goal(-0.15)
        
    # 컨베이어 벨트 위로 옮기기 함수
    def drop_con(self):
        self.move_arm_to_position(0, 200, 90)
        time.sleep(2)
        self.send_gripper_goal(0.025)
        time.sleep(2)
        
        # 작업 중 표시 초기화
        self.ignore = False
        
        # 컨베이어 벨트 이동 퍼블리시
        self.drop_pub.publish(Int32(data = 1))    
    
    # 매니퓰레이터 동작 함수
    def move_arm_to_position(self, x, y, z):
        self.J0, self.J1, self.J2, self.J3, self.Sxy, self.sr1, self.sr2, self.sr3, self.St, self.Rt = self.solv_robot_arm2(x, y, z, self.r1, self.r2, self.r3)

        self.trajectory_msg = JointTrajectory()
        self.trajectory_msg.header = Header()
        self.trajectory_msg.header.frame_id = 'base_link'
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = [self.Sxy, self.sr1 + self.th1_offset, self.sr2 + self.th2_offset, self.sr3]
        point.velocities = [0.0] * 4
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 5000

        # 매니퓰레이터 각도 설정
        self.trajectory_msg.points = [point]
        
        # 매니퓰레이터 동작 토픽 발행
        self.joint_pub.publish(self.trajectory_msg)    
    
    # 매니퓰레이터 정보 계산 함수
    def solv_robot_arm2(self, x, y, z, r1, r2, r3):
        Rt = math.sqrt(x**2 + y**2 + z**2)
        Rxy = math.sqrt(x**2 + y**2)
        St = math.asin(z / Rt)
        Sxy = math.atan2(y, x)
        s1, s2 = self.solv2(r1, r2, Rt)

        sr1 = math.pi / 2 - (s1 + St)
        sr2 = s1 + s2
        sr2_ = sr1 + sr2
        sr3 = math.pi - sr2_

        J0 = (0, 0, 0)
        J1 = (J0[0] + r1 * math.sin(sr1) * math.cos(Sxy),
            J0[1] + r1 * math.sin(sr1) * math.sin(Sxy),
            J0[2] + r1 * math.cos(sr1))
        J2 = (J1[0] + r2 * math.sin(sr1 + sr2) * math.cos(Sxy),
            J1[1] + r2 * math.sin(sr1 + sr2) * math.sin(Sxy),
            J1[2] + r2 * math.cos(sr1 + sr2))
        J3 = (J2[0] + r3 * math.sin(sr1 + sr2 + sr3) * math.cos(Sxy),
            J2[1] + r3 * math.sin(sr1 + sr2 + sr3) * math.sin(Sxy),
            J2[2] + r3 * math.cos(sr1 + sr2 + sr3))

        return J0, J1, J2, J3, Sxy, sr1, sr2, sr3, St, Rt
    
    # 링크 각도 계산 함수
    def solv2(self, r1, r2, r3):
        d1 = (r3**2 - r2**2 + r1**2) / (2 * r3)
        d2 = (r3**2 + r2**2 - r1**2) / (2 * r3)
        s1 = math.acos(d1 / r1)
        s2 = math.acos(d2 / r2)
        return s1, s2             


def main(args=None):
    rclpy.init(args=args)
    
    arm_controller = ArmController() 
    rclpy.spin(arm_controller)
    arm_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
