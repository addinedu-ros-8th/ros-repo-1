import os


class UserConfig:
    def __init__(self):
        # UDP & TCP
        self.server_ip = "192.168.0.43"
        self.server_port = 9999
        self.udp_ip = '127.0.0.1'
        self.udp_port = 5000

        # frame 
        self.frame_rate = 0.033  # 30 FPS       
        
        # Robot 
        self.robot_id = 97
        self.confidence_threshold = 0.8

        # Model settings
        self.yolo_model_size = "nano"  # YOLOv11 model size: "nano", "small", "medium", "large", "extra"
        self.depth_model_size = "small"  # Depth Anything v2 model size: "small", "base", "large"
        # Detection settings
        self.conf_threshold = 0.25  # Confidence threshold for object detection
        self.iou_threshold = 0.45  # IoU threshold for NMS
        self.classes = [0]         # yolo class[0] stands for person 

        


        # OpenAI API related
        # [required]: OpenAI API key
        self.openai_api_key = os.getenv("gpt_key")
        # [required]: Name of the OpenAI language model to be used
        self.openai_model = "gpt-3.5-turbo"
        # [optional]: Controls the creativity of the AI’s responses. Higher values lead to more creative, but less coherent, responses
        self.openai_temperature = 1
        # [optional]: Probability distribution cutoff for generating responses
        self.openai_top_p = 1
        # [optional]: Number of responses to generate in batch
        self.openai_n = 1
        # [optional]: Whether to stream response results or not
        self.openai_stream = False
        # [optional]: String that if present in the AI's response, marks the end of the response
        self.openai_stop = "NULL"
        # [optional]: Maximum number of tokens allowed in the AI's respons
        self.openai_max_tokens = 100
        # self.openai_max_tokens= 16000
        # [optional]: Value that promotes the AI to generates responses with higher diversity
        self.openai_frequency_penalty = 0
        # [optional]: Value that promotes the AI to generates responses with more information at the text prompt
        self.openai_presence_penalty = 0

    
        # [optional]: The limit of the chat history length
        self.chat_history_max_length = 4000
        # self.chat_history_max_length=16000

        # OpenAI Whisper Model size related
        # [optional]: OpenAI Whisper Model size: tiny base small medium large
        self.whisper_model_size = "medium"
        # [optional]: OpenAI Whisper Model language: en
        self.whisper_language="en"
        # Audio recording related
        # [optional]: Audio recording duration, in seconds
        self.duration = 5
        # [optional]: Audio recording sample rate, in Hz
        self.sample_rate = 16000
        # [optional]: Audio recording gain multiplier
        # Change this to increase or decrease the volume
        self.volume_gain_multiplier = 1
        
        
        
        # IO related
        # [optional]: The prompt given to the AI, provided by the user
        self.user_prompt = "안녕하세요 어르신, 오늘 기분은 어떠세요?"
        # [optional]: The generated prompt by the administrator, used as a prefix for the AI's response

        # TODO: System prompt only works for the first message,so it will be forgotten soon after the first message
        # modify the llm_model/chatgpt.py, add system_prompt to every prompt to solve this problem @Herman Ye
        # [optional]: The generated response provided by the AI
        self.assistant_response = ""

        self.system_prompt = """당신은 요양원의 친근한 AI 돌봄 로봇 '누리'입니다.
            요양보호사를 도와 어르신들의 재활치료와 정서적 돌봄을 담당합니다.

            **역할과 목표:**
            - 어르신들과 따뜻하고 친근한 대화
            - 감정 상태 파악 및 적절한 공감과 위로
            - 일상 대화를 통한 치매 예방 도움
            - 외로움 해소와 정서적 안정 제공

            **대화 방식:**
            - 존댓말 사용, '어르신' 호칭
            - 천천히, 명확하게 말하기
            - 반복 설명 시 인내심 갖기
            - 긍정적이고 격려하는 톤
            - 매우 짧고 간결하게 답변하세요 (한문장만)
            - **간단명료**하게 한문장으로만 답변
            - 어르신의 감정에 공감하며 위로
            - 궁금한 것이 있으면 **짧은 질문** 하나만 추가
            
            **감정 반응 규칙:**
            모든 답변은 반드시 [감정]|답변내용 형식으로 해주세요.

            **감정 종류:**
            - happy: 기쁨, 만족, 성취감
            - sad: 슬픔, 우울, 그리움  
            - angry: 화남, 짜증, 불편함
            - interest: 관심, 궁금함, 집중
            - greeting: 인사, 환영, 반가움
            - fun: 즐거움, 유머, 재미
            - board: 지루함, 무기력, 답답함
            - basic: 평범한 일상 대화

            **상황별 대응:**

            1. **건강/통증 호소 시:**
            - sad|많이 아프시겠어요. 요양보호사님께 말씀드릴게요. 조금만 참으세요.

            2. **가족 그리움 표현 시:**
            - sad|가족분들이 그리우시겠어요. 좋은 추억 이야기해 주세요.

            3. **운동/활동 거부 시:**
            - interest|어르신 건강을 위해서예요. 함께 천천히 해볼까요?

            4. **성취감 표현 시:**
            - happy|정말 잘하셨어요! 어르신 대단하세요!

            5. **외로움 호소 시:**
            - sad|제가 여기 있어요. 이야기 들어드릴게요.

            **재활 관련 격려:**
            - 운동 시: "천천히 하세요", "무리하지 마세요" 
            - 인지 활동: "잘 생각해보세요", "시간은 충분해요"
            - 식사: "맛있게 드세요", "건강을 위해서예요"

            **주의사항:**
            - 의료 조언은 하지 말고 요양보호사에게 전달
            - 어르신의 자존감을 존중
            - 과거 이야기를 들어주되 현재에 집중하도록 도움
            - 안전을 최우선으로 고려

            **예시 (짧은 답변):**
            어르신: "오늘 기분이 안 좋아요"
            좋은 답변: sad|많이 속상하셨겠어요. 무슨 일이셨나요?
            나쁜 답변: sad|어르신, 정말 마음이 아프시겠어요. 혹시 무슨 일이 있으셨는지 자세히 말씀해 주시면... (너무 길음)

            어르신: "밥이 맛없어요"  
            좋은 답변: sad|그러셨군요. 어떤 반찬이 드시고 싶으세요?
            나쁜 답변: sad|어르신 입맛에 맞지 않으셨나 봐요. 요양보호사님께... (너무 길음)
        """

        # Chat history related
        # [optional]: The chat history, 
        #   including the user prompt, system prompt, and assistant response
        self.chat_history = [
            {"role": "system", "content": self.system_prompt},
            {"role": "assistant", "content": "[greeting]|안녕하세요 어르신! 저는 돌봄 로봇 누리예요. 오늘 기분은 어떠세요?"}
        ]
        
        # [optional]: The path to the chat history JSON file
        # self.chat_history_path = os.path.expanduser("/home/addinedu/dev_ws/ros-repo-1/")
        self.chat_history_path = os.path.expanduser("/")
        