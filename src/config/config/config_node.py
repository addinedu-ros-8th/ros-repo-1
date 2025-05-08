import os


class UserConfig:
    def __init__(self):
        # udp
        self.server_ip = "192.168.0.43 "
        self.server_port = 9999
        self.frame_rate = 0.033  # 30 FPS
        # robot 
        self.robot_ip = 1
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
        self.openai_max_tokens = 500
        # self.openai_max_tokens= 16000
        # [optional]: Value that promotes the AI to generates responses with higher diversity
        self.openai_frequency_penalty = 0
        # [optional]: Value that promotes the AI to generates responses with more information at the text prompt
        self.openai_presence_penalty = 0

        # IO related
        # [optional]: The prompt given to the AI, provided by the user
        self.user_prompt = "당신은 요양원에 있는 노인입니다"
        # [optional]: The generated prompt by the administrator, used as a prefix for the AI's response
        self.system_prompt = "당신은 요양원에서 노인의 대화 상대입니다."
        # TODO: System prompt only works for the first message,so it will be forgotten soon after the first message
        # modify the llm_model/chatgpt.py, add system_prompt to every prompt to solve this problem @Herman Ye
        # [optional]: The generated response provided by the AI
        self.assistant_response = ""

        # Chat history related
        # [optional]: The chat history, including the user prompt, system prompt, and assistant response
        self.chat_history = [{"role": "system", "content": self.system_prompt}]
        # [optional]: The path to the chat history JSON file
        # self.chat_history_path = os.path.expanduser("/home/addinedu/dev_ws/ros-repo-1/")
        self.chat_history_path = os.path.expanduser("/")
        
        # [optional]: The limit of the chat history length
        self.chat_history_max_length = 4000
        # self.chat_history_max_length=16000

        # # AWS related
        # # [required]: AWS IAM access key id
        # self.aws_access_key_id = os.getenv("AWS_ACCESS_KEY_ID")
        # # [required]: AWS IAM secret access key
        # self.aws_secret_access_key = os.getenv("AWS_SECRET_ACCESS_KEY")
        # # [required]: AWS IAM region name
        # self.aws_region_name = 'ap-southeast-1'
        # # [required]: AWS S3 bucket name
        # self.bucket_name = 'auromixbucket'
        # self.aws_transcription_language = "en-US"
        # # [optional]: AWS polly voice id, change this to 'Zhiyu' for Chinese
        # self.aws_voice_id = "Ivy"

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