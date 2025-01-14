import rclpy, time, math
from rclpy.node import Node
from std_msgs.msg import Int32, Header, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

# 로봇팔 거리계산
r1 = 130
r2 = 124
r3 = 126
th1_offset = - math.atan2(0.024, 0.128)
th2_offset = - 0.5 * math.pi - th1_offset

def solv2(r1, r2, r3):
    d1 = (r3**2 - r2**2 + r1**2) / (2 * r3)
    d2 = (r3**2 + r2**2 - r1**2) / (2 * r3)
    s1 = math.acos(d1 / r1)
    s2 = math.acos(d2 / r2)
    return s1, s2

def solv_robot_arm2(x, y, z, r1, r2, r3):
    Rt = math.sqrt(x**2 + y**2 + z**2)
    Rxy = math.sqrt(x**2 + y**2)
    St = math.asin(z / Rt)
    Sxy = math.atan2(y, x)
    s1, s2 = solv2(r1, r2, Rt)

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

joint_angle_delta = 0.1  # radian

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.box_position = 0
        self.ignore = False
        
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')
        self.drop_pub = self.create_publisher(Int32,'drop_condition',10)
        self.create_subscription(Int32,'stop',self.stop_callback,10)
        # box_condition 토픽 구독
        self.create_subscription(String, 'box_condition', self.box_condition, 10)

        self.blue_count = self.create_publisher(Int32,'blue_num',10)
        self.red_count = self.create_publisher(Int32,'red_num',10)
        self.create_subscription(Int32,'arrive',self.arrive_callback,10)
        self.capture_publisher = self.create_publisher(Int32,'capture',10)
        self.subscription__all_move = self.create_subscription(Int32,'all_move',self.all_move_callback,10)
        self.trajectory_msg = JointTrajectory()
        
        # 현재 각 색깔 몇 개 이동시킨지 카운트
        self.red = 0
        self.blue = 0
        # 초기 로봇 팔 위치 설정
        J0, J1, J2, J3, Sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(100, 0, 100, r1, r2, r3)
        self.trajectory_msg.header = Header()
        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        point = JointTrajectoryPoint()
        point.positions = [Sxy, sr1 + th1_offset, sr2 + th2_offset, sr3]
        point.velocities = [0.0] * 4
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 5000
        self.trajectory_msg.points = [point]
        self.move_count = 0
        self.all_move = 0
        self.detected_color = None
        self.arrive = 0     
        self.arrive_c = 0
        self.stop = 0


        #self.move()
    def stop_callback(self,msg):
        self.stop = msg.data
        if self.stop_move == 1:
            self.destroy_node()  # 노드를 종료
            rclpy.shutdown()  # rclpy를 종료하여 노드를 정리합니다.

    def all_move_callback(self,msg):
        self.all_move = msg.data

    def arrive_callback(self,msg):
        self.arrive = msg.data
        if self.arrive ==1 and self.arrive_c == 0:
            self.mani_home()
            # 그리퍼 초기화
            self.send_gripper_goal(0.025)
            self.arrive_c += 1
            self.move()

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
            self.trajectory_msg.points[0].positions[0]+=0.2
            self.joint_pub.publish(self.trajectory_msg)
            time.sleep(1)
            self.move_arm_to_position(0,40,230)
            time.sleep(1)
            self.move_arm_to_position(0,-40,230)
        
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

            

    def move_arm_to_position(self, x, y, z):
        # inverse kinematics 계산
        J0, J1, J2, J3, Sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(x, y, z, r1, r2, r3)

        self.trajectory_msg = JointTrajectory()
        self.trajectory_msg.header = Header()
        self.trajectory_msg.header.frame_id = 'base_link'
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = [Sxy, sr1 + th1_offset, sr2 + th2_offset, sr3]
        point.velocities = [0.0] * 4
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 5000

        self.trajectory_msg.points = [point]
        self.joint_pub.publish(self.trajectory_msg)

    def move(self):
        # box_position이 업데이트될 때까지 기다리기 위한 타이머 설정
        self.timer = self.create_timer(0.1, self.check_box_position)

    def check_box_position(self):
        # box_position이 0이 아니고 ignore가 False일 때만 작업 시작
        if self.box_position != 0 and not self.ignore:
            self.timer.cancel()  # 타이머 종료
            if self.detected_color == 'Red':
                self.red+=1
                self.red_count.publish(Int32(data = self.red))
                self.get_logger().info(f'published RED count {self.red}')
            elif self.detected_color == 'Blue':
                self.blue+=1
                self.blue_count.publish(Int32(data = self.blue))
                self.get_logger().info(f'published BLUE count {self.blue}')

            self.ignore = True  # Prevent further interference
            self.get_logger().info(f'좌표 확인 된 곳: {self.box_position}')
            self.capture_publisher.publish(Int32(data=1))
            self.capture_publisher.publish(Int32(data=0))
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

            self.capture_publisher.publish(Int32(data=2))
            self.capture_publisher.publish(Int32(data=0))
            self.get_logger().info(f'자리 확인: {self.box_position}')
            self.trajectory_msg.points[0].positions[1] -= 0.5
            self.joint_pub.publish(self.trajectory_msg)
            time.sleep(2)
            self.drop_con()
            self.mani_home()
            self.box_position = 0
            self.detected_color = " "
                
            self.get_logger().info(f'박스 위치: {self.box_position}    박스 색: {self.detected_color}')
            self.timer = self.create_timer(0.1, self.check_box_position)

    def grip_one(self):
        self.move_arm_to_position(145, 60, 50)
        time.sleep(2)
        self.get_logger().info('1번 자리 그립')
        self.send_gripper_goal(-0.15)
        

    def grip_two(self):
        self.move_arm_to_position(145, -70, 50)
        time.sleep(2)
        self.get_logger().info('2번 자리 그립')
        self.send_gripper_goal(-0.15)
        
    
    def grip_three(self):
        self.move_arm_to_position(200, 65, 50)
        time.sleep(2)
        self.get_logger().info('3번 자리 그립')
        self.send_gripper_goal(-0.15)
        #1000*90

    def grip_four(self):
        self.move_arm_to_position(200, -70, 50)
        time.sleep(2)
        self.get_logger().info('4번 자리 그립')
        self.send_gripper_goal(-0.15)
        

    def drop_con(self):
        self.move_arm_to_position(0, 200, 90)
        time.sleep(2)
        self.send_gripper_goal(0.025)
        time.sleep(2)
        self.ignore = False  # Allow the next action
        #아두이노 이동용
        self.drop_pub.publish(Int32(data = 1))

    def send_gripper_goal(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = -1.0
        self.gripper_action_client.send_goal_async(goal)
        time.sleep(2)

    def box_condition(self, msg):
        if not self.ignore:
            try:
            # 구역 및 색상 정보 파싱
                detected_info = msg.data
                if detected_info == 'go_purple':
                    return
                # "Zone X - Color" 형식의 문자열 파싱
                else:
                    parts = detected_info.split(" - ")
                    if len(parts) == 2:
                        self.box_position = int(parts[0].split(" ")[1])  # "Zone X"에서 X 추출
                        self.detected_color = parts[1]  # 색상 추출
            except Exception as e:
                self.get_logger().error(f"Failed to parse detected info: {str(e)}")

    def mani_home(self):
        if self.move_count >= self.all_move:
            self.trajectory_msg.points[0].positions[0] = 0.0
            self.trajectory_msg.points[0].positions[1] = -0.845402829343338
            self.trajectory_msg.points[0].positions[2] = 0.47597600165170617
            self.trajectory_msg.points[0].positions[3] = 1.4402231544865285            
            self.joint_pub.publish(self.trajectory_msg)
            self.get_logger().info('과정 종료시 매니퓰레이터 홈포지션')
        else:
            self.trajectory_msg.points[0].positions[0] = 0
            self.trajectory_msg.points[0].positions[1] = -1.445402
            self.trajectory_msg.points[0].positions[2] = 0.8759760016517062
            self.trajectory_msg.points[0].positions[3] = 1.1402231544865281
            self.joint_pub.publish(self.trajectory_msg)
            self.get_logger().info('매니퓰레이터 홈포지션')
        self.move_count +=1
        time.sleep(5)

def main():
    rclpy.init()  # ROS 2 초기화
    arm_controller = ArmController()  # 구독자 노드 객체 생성
    rclpy.spin(arm_controller)  # 노드가 종료될 때까지 실행
    arm_controller.destroy_node()  # 노드 종료
    rclpy.shutdown()  # ROS 2 종료

if __name__ == '__main__':
    main()