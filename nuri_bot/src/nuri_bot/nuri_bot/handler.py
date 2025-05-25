from std_msgs.msg import String

import math
import time 
from geometry_msgs.msg import Twist, PoseStamped
from transforms3d.euler import quat2euler

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient 

class Handler:

    def __init__(self, node, cam, stt, tts) :
        # 사전 정의된 산책 경로 포인트
        self.waypoints = [
            (1.17, -0.29),      # second
            (0.371, -0.29),     # third 
            (0.371, 0.443),     # forth
            (1.17, 0.443)       # first 
        ]

        self.node = node
        self.cam = cam 
        self.stt = stt 
        self.tts = tts

        # -- publishers --
        self.__sub_schedule = self.node.create_subscription(         # main server로 부터 '일정'
            String,'/robot1/schedule', self.schedule_callback, 10)    
        
        self.__pub_for_positive = self.node.create_publisher(        # main server에 긍정적 답변 전달
            String, "/walk_positive", 10)
        self.__pub_cmd_vel = self.node.create_publisher(             # cmd_vel 토픽 발행자 추가
            Twist, "/cmd_vel", 10)   

        # -- subscribers -- 
        self.__sub_for_arrive = self.node.create_subscription(       # main server로 부터 '도착' 
            String, "/arrive", self.arrive_callback, 10)
            
        self.__sub_for_closefar = self.node.create_subscription(     # ai_server로 부터 'close or far 응답'
            String, "/report_closefar", self.distance_callback, 10)
        


        self.__sub_response_complete = self.node.create_subscription(  # stt_node로 부터 '응답'
            String, "/response_walk", self.response_complete_callback, 10)

        self.__sub_tracked_pose = self.node.create_subscription(
            PoseStamped, "/tracked_pose", self.pose_callback, 10)
        
        self.__sub_speech_complete = self.node.create_subscription(  # tts_node로 부터 '말하기 완료'
            String, "/speech_complete", self.speech_complete_callback, 10)
        
        self.__sub_bye = self.node.create_subscription(  # tts_node로 부터 '말하기 완료'
            String, "/bye", self.bye_callback, 10)

        self.response_timer = None
        self.node.get_logger().info("Handler 초기화 완료.") # 추가
        
        self.temp_name = None

        # 웨이포인트 네비게이션 관련 변수들
        self.current_index = 0           # 현재 웨이포인트 인덱스 (0에서 시작)
        self.is_walking = False          # 초기에는 산책 중이 아님
        self.distance_threshold = 0.2    # 도착 판단 거리 임계값 (m) - 30cm로 현실적으로
        self.walk_speed = 0.2            # 기본 이동 속도 (m/s)
        self.completed_waypoints = 0     # 완료한 웨이포인트 개수 추적 - 초기화 추가!

        # 회전 관련 변수들
        self.is_rotating = False         # 회전 중인지 여부
        self.rotation_timer = None       # 회전 타이머
        self.rotation_duration = 3.14    # 90도 회전 시간 (π/2 / 0.5 = 약 3.14초)
        
        # 현재 위치 초기화
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.twist = Twist()
        
        self.action_client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.scenairo("Walk")
        self.concept = None 


    
    def create_pose_goal(self, x, y, yaw=0.0):
        """PoseStamped 목표 생성"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'  # 또는 'base_link'
        goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0  # 회전 없음
        return goal_pose

    def scenairo(self, schedule = "Standby"):
        """ Debug """
        if schedule == "Talk":
            self.stt.run()
            self.concept = "Talk"
        
        elif schedule == "Walk":
            self.cam.set_current_cmd("Walk")
            self.start_walk()
            self.concept = "Walk"

        elif schedule == "Standby":
            self.cam.set_current_cmd("Standby")
            self.concept="Standby"
    
    def start_walk(self):
        """산책 시작"""
        self.node.get_logger().info("산책 시작!")
        self.is_walking = True
        
        # init 
        self.current_index = 0        # 웨이포인트 3에서 시작
        self.completed_waypoints = 0  # 완료 카운터 초기화
        
        target_x, target_y = self.waypoints[0]
        # self.start_moving()
        # action 목표 설정 
        # goal_msg = NavigateToPose.Goal()
        # goal_msg.pose = self.create_pose_goal(target_x, target_y)

        # send_goal_future = self.action_client.send_goal_async(goal_msg)
        # send_goal_future.add_done_callback(self.get_result_callback)

        self.node.get_logger().info(f"목표 지점: ({target_x:.2f}, {target_y:.2f})")

    def start_moving(self):
        """목표 방향으로 이동"""

        # 다음 목표 웨이포인트 계산
        next_index = (self.current_index) % len(self.waypoints)
        target_x, target_y = self.waypoints[next_index]
        
        dx = target_x - self.current_x
        dy = target_y - self.current_y 

        distance = math.sqrt(dx**2 + dy**2) 

        cmd = Twist()  # 괄호 추가!
        speed = self.walk_speed
        z_speed = 0.0

        movement_axis = (next_index + 2) % 2

        # print(abs(dx), abs(dy), self.current_x, self.current_y, self.nro , flush=True)

        if movement_axis == 0:  # Y축 이동
            # self.node.get_logger().info(f"Y축 이동 모드 {abs(dy)}")
            if abs(dy) <= self.distance_threshold :
                # cmd.angular.z = 0.0
                speed = 0.0
                if not self.is_rotating:
                    self.start_rotation()
            # else:
            #     cmd.linear.x = 0.0
            #     cmd.angular.z = 0.0
            #     self.node.get_logger().info("Y축 정렬")
            
        else:  # X축 이동 (movement_axis == 0)
            # self.node.get_logger().info(f"X축 이동 모드 {abs(dx)}")  
            if self.current_index == 3:
                self.distance_threshold = 0.1
            if abs(dx) <= self.distance_threshold:
                speed = 0.0
                # cmd.angul,ar.z = 0.0
                if not self.is_rotating:
                    self.start_rotation()
            # else:
            #     # X축 정렬됨
            #     cmd.angular.z = 0.0
            #     self.node.get_logger().info("X축 정렬")

        cmd.linear.x = speed
        # cmd.angular.z = z_speed  # 이동 중에는 회전 없음
        self.__pub_cmd_vel.publish(cmd)

    def check_waypoint_arrival(self):
        """waypoint 도착 체크"""
        if not self.is_walking or self.is_rotating: 
            return
            
        # 다음 목표 웨이포인트와의 거리 계산
        next_index = (self.current_index) % len(self.waypoints)
        target_x, target_y = self.waypoints[next_index]
        dx = target_x - self.current_x
        dy = target_y - self.current_y 
        distance = math.sqrt(dx**2 + dy**2)
        
        # 웨이포인트 도착 체크
        if distance < 0.4:
            self.node.get_logger().info(f" 웨이포인트 {next_index} 도착! (거리: {distance:.2f})")
            self.node.get_logger().info(f" 현재위치: ({self.current_x:.2f}, {self.current_y:.2f})")
            self.node.get_logger().info(f" 목표위치: ({target_x:.2f}, {target_y:.2f})")
            # 90도 시계방향 회전 시작
            self.start_rotation()
           
            # 현재 위치를 도착한 웨이포인트로 업데이트
            # self.current_index = next_index
            
            # 완료한 웨이포인트 개수 증가
            self.completed_waypoints += 1
            self.node.get_logger().info(f'previous waypoint : {self.current_index}, current waypoint: {self.completed_waypoints}') 
            self.current_index = self.completed_waypoints
            
            # 1차 순환 완료 체크 (0->1->2->3->0 = 4개 이동 완료)
            if self.completed_waypoints >= len(self.waypoints):  # 4개 이동 완료
                self.node.get_logger().info(" 1차 순환 완료! 산책 종료")
                self.is_walking = False
                self.stop_robot()
                return      

    def start_rotation(self):
        """웨이포인트 도착 후 90도 시계방향 회전"""
        self.node.get_logger().info(" 90도 시계방향 회전 시작")
        try:
            if self.current_index == 3:
                self.node.get_logger().info(" 1차 순환 완료! 산책 종료")
                self.is_walking = False
                return

            self.is_rotating = True
            
            twist = Twist()
            twist.angular.z = -0.3
            self.__pub_cmd_vel.publish(twist)

            time.sleep(1)
            
            self.stop_robot()
            time.sleep(1)
            self.is_rotating = False
            
            # 다음 목표 안내
            self.completed_waypoints += 1
            self.current_index = self.completed_waypoints
        except Exception as e:
            print(e, flush=True)
        
        # 회전 완료 타이머 설정
        exact_90_time = (math.pi / 2) / 0.4
        # self.rotation_timer = self.node.create_timer(1, self.finish_rotation)
        # self.rotation_timer = self.node.create_timer(self.rotation_duration, self.finish_rotation)

    def check_rotation(self):
        error = self.norm_angle(self.norm_angle(self.current_yaw + math.radians(90)) - self.current_yaw)

        self.twist.angular.z = -0.2
        self.__pub_cmd_vel.publish(self.twist)
        print(error, flush=True)
        if abs(error) < 0.05:
            print("회전 완료", flush=True)
            self.stop_robot()
            self.is_rotating = False
            self.rotation_timer.cancel()
            self.finish_rotation = None
            return
        
    def finish_rotation(self):
        """회전 완료 처리"""
        self.node.get_logger().info(" 90도 회전 완료")
        self.is_rotating = False
        
        # 타이머 정리
        if self.rotation_timer is not None:
            self.rotation_timer.cancel()
            self.rotation_timer = None
        
        # 로봇 정지
        self.stop_robot()
        
        # 다음 목표 안내
        next_next_index = (self.current_index + 1) % len(self.waypoints)
        next_target_x, next_target_y = self.waypoints[next_next_index]

        self.node.get_logger().info(f" 다음 목표: 웨이포인트 {next_next_index} ({next_target_x:.2f}, {next_target_y:.2f})")
        self.node.get_logger().info(f"   완료 진행도: {self.completed_waypoints}/{len(self.waypoints)}")
    

    def stop_robot(self):
        """로봇 정지 명령 발행"""
        # self.node.get_logger().info(" 로봇 정지")
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        self.__pub_cmd_vel.publish(cmd)


    # ========================= callback ====================
    def schedule_callback(self, msg):
        self.node.get_logger().info("Received schedule: '%s'" % msg.data)
        """
            schedule : Standby, Walk
        """
        schedule = msg.data

        try: 
            self.cam.set_current_cmd(schedule)
        except ValueError:
            self.node.get_logger().error(f"잘못된 데이터수신: {msg.data}") 


    def goal_response_callback(self, future): 
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.node.get_logger().info("goal rejected")
            return 
        self.node.get_logger().info("goal accepted")

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """ 시작 waypoint 0부터 1 시작"""
        result = future.result().result
        self.node.get_logger().info(f"네비게이션 완료: {result}")

    def response_complete_callback(self, msg):
        """요양원 어르신 응답에 맞는 처리"""
        response_text = msg.data
        self.node.get_logger().info(f"어르신 응답: {response_text}")
        
        emergency_keywords = ["아파", "도와줘", "숨이", "가슴이", "머리가", "어지러워"]
        if any(keyword in response_text for keyword in emergency_keywords):
            self.node.get_logger().warning("응급 상황 감지! 요양보호사 호출 필요")
        
        if self.concept == "Talk":
            self.node.get_logger().info("듣는 중 ")
        
        elif self.concept == "Walk":
            if  "응" in response_text or \
                "네" in response_text or \
                "가자" in response_text:
                self.node.get_logger().info("산책 수락")
                self.tts.tts_speak("네, 천천히 함께 걸어요. 무리하지 마세요!", "next")

                positive_msg = String()
                positive_msg.data = "positive"
                self.__pub_for_positive.publish(positive_msg)
                self.node.get_logger().info("발행됨: /walk_positive 토픽에 'positive' 메시지")
                
            elif "아니" in response_text or \
                "안해" in response_text or \
                "싫어" in response_text:
                self.node.get_logger().info("산책 거절")
                self.tts.tts_speak("어르신, 좋은 말 할때 산책가시죠.", "again")
                
            else:
                self.node.get_logger().info("불명확한 응답, 다시 물어보기")
                self.tts.tts_speak("죄송합니다. 다시 한번 대답해 주세요.", "walk")


    def arrive_callback(self, msg):
        self.node.get_logger().info("Received arrive msg: '%s'" % msg.data)
        
        parts = msg.data.split(',')
        if len(parts) == 2:         # 침대로 도착 (arrive,나덕윤)
            msg_type = parts[0]
            name = parts[1]
            self.temp_name = name
            self.node.get_logger().info(f"Arrived at which {name} stays")

            # emotion 
            # req = Emotion.Request()
            # req.emotion = "hello"
            # self.__srv_emotion.call_aync(req)

            # greeting 
            text = f"안녕하세요 {name} 어르신, 산책 가실 시간이에요. 산책가시죠."
            self.tts.tts_speak(text, "walk")
                
        else:                       # 산책경로 1로 도착  (arrive)
            self.node.get_logger().info("Arrived at position 1 for walk")
            self.cam.set_current_cmd('Walk')

    def pose_callback(self, msg):
        """ 로봇의 현재 위치 업데이트"""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        
        self.yaw = self.get_yaw(msg)

        # self.node.get_logger().debug(f"현재위치: ({self.current_x:.2f}, {self.current_y:.2f})")
        # self.node.get_logger().info(f"현재위치: ({self.current_x:.2f}, {self.current_y:.2f})") # ros2의 기본 로그레벨은 info이상이라서 debug메시지는 출력되지 않습니다
        
        # if self.is_walking:
        #     self.check_waypoint_arrival()

    def get_yaw(self, msg):
        orientation = msg.pose.orientation
        q = [orientation.w, orientation.x, orientation.y, orientation.z]
        _, _, yaw = quat2euler(q)

        return yaw

    def norm_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def distance_callback(self, msg):
        """ close/far 상태에 따라 한 번만 시작/정지"""
        if not self.is_walking: 
            return 
        
        if msg.data == "close":
            # self.node.get_logger().info("getting close, keep going!")    
            # 회전 중이 아닐 때만 이동
            if not self.is_rotating:
                self.start_moving()    
        else: 
            # self.node.get_logger().info("too far!")
            self.stop_robot()
            # 회전 타이머도 중지
            if self.rotation_timer is not None:
                self.rotation_timer.cancel()
                self.rotation_timer = None
                self.is_rotating = False

    
    def bye_callback(self, msg):
        self.node.get_logger().info("그만 듣자")
        self.tts.tts_speak("헤헤, 필요하시면 또 불러주세요!", "bye")

    def speech_complete_callback(self, msg):
        speech_type = msg.data
        self.node.get_logger().info(f"응답 완료, {speech_type}")

        if speech_type == "spoken":
            self.node.get_logger().info("계속 듣자")
            self.stt.action_function_listening()
        else:
            self.node.get_logger().info("그만 듣자")