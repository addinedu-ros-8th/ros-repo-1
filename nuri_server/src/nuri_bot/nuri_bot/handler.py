


# Debug - arrive : ros2 topic pub /arrive std_msgs/msg/String "data: '나덕윤'" -1


# ros2 topic pub /report_closefar std_msgs/msg/String "data: 'close'"
#ros2 topic pub /report_closefar std_msgs/msg/String "data: 'far'"

# 한 번만 보내기 (--once 옵션)
#ros2 topic pub --once /report_closefar std_msgs/msg/String "data: 'close'"

# 1Hz로 반복해서 보내기 (--rate 옵션)
#ros2 topic pub --rate 1 /report_closefar std_msgs/msg/String "data: 'close'"
from std_msgs.msg import String

class Handler:

    def __init__(self, node, cam, stt, tts) :
        self.node = node
        self.cam = cam 
        self.stt = stt 
        self.tts = tts

        # -- subscribers -- 
        self.__sub_schedule = self.node.create_subscription(         # main server로 부터 '일정'
            String,'/robot1/schedule', self.schedule_callback, 10)    
        
        self.__sub_for_arrive = self.node.create_subscription(       # main server로 부터 '도착' 
            String, "/arrive", self.arrive_callback, 10)
        self.__sub_for_closeFar = self.node.create_subscription(
            String, "/report_closefar", self.distance_callback 
        )
        
        self.__sub_speech_complete = self.node.create_subscription(  # tts_node로 부터 '말하기 완료'
            String, "/speech_complete", self.speech_complete_callback, 10)

        self.__sub_response_complete = self.node.create_subscription(  # stt_node로 부터 '산책 응답'
            String, "/response_walk", self.response_complete_callback, 10)

        self.temp_x = -1    # store the target person's x-coord 
        self.temp_y = -1    # store the target person's y-coord


        self.response_timer = None
        self.node.get_logger().info("Handler 초기화 완료.") # 추가

        self.temp_name = None
        
    def schedule_callback(self, msg):
        self.node.get_logger().info("Received schedule: '%s'" % msg.data)
        """
            schedule : Standby, Walk

            msg args: type, x, y 
        """
        parts = msg.data.split(',')
        if len(parts) == 3:
            cmd = parts[0]
            self.temp_x = parts[1]
            self.temp_y = parts[2]
            try: 
                self.cam.set_current_cmd(cmd)

            except ValueError:
                self.node.get_logger().error(f"잘못된 데이터수신: {msg.data}")  
        else:
            self.node.get_logger().warn('Could not read frame')

    def speech_complete_callback(self, msg):
        speech_type = msg.data
        if speech_type == "walk":
            self.node.get_logger().info(f"말하기 완료, shall we {speech_type}..?")
            self.stt.action_function_listening(speech_type)

        elif speech_type == "next":
            self.node.get_logger().info(f"응답 완료, {self.temp_name} {speech_type}ed,\
                                        start having a walk")
            # 주행 
        elif speech_type == "again": 
            self.node.get_logger().info(f"응답 완료, ask {speech_type} to {self.temp_name} ,\
                            start having a walk")

    # =============================  산책 ===================================== 
    def arrive_callback(self, msg):
        if msg.data:
            name = msg.data.strip()
            self.temp_name = name 
            self.node.get_logger().info(f"Arrived at which {msg.data} stays")
            
            # greeting 
            text = f"안녕하세요 {name} 어르신, 산책 가실 시간이에요. 산책가시죠."
            self.tts.tts_speak(text, "greeting")

    def response_complete_callback(self, msg):
        """산책 제안에 대한 응답 처리"""
        response_text = msg.data
        self.node.get_logger().info(f"산책 응답 처리: {response_text}")
        
        if  "응" in response_text or \
            "네" in response_text or \
            "가자" in response_text:
            self.node.get_logger().info("산책 수락")
            self.tts.tts_speak("네, 산책을 시작하겠습니다. 저를 따라서 걸어요!", "accept")
            
        elif "아니" in response_text or \
             "안해" in response_text or \
             "싫어" in response_text:
            self.node.get_logger().info("산책 거절")
            self.tts.tts_speak("어르신, 좋은 말 할때 산책가시죠.", "deny")
            
        else:
            self.node.get_logger().info("불명확한 응답, 다시 물어보기")
            self.tts.tts_speak("죄송합니다. 다시 한번 대답해 주세요.", "greeting")
            

    
