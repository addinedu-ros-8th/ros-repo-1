# llm_output/llm_output/lcd_controller.py
from .pinky_lcd import LCD
from PIL import Image, ImageSequence
import time
import os

class LCDController:
    def __init__(self, node_logger=None):
        self.logger = node_logger if node_logger else print # 이 줄은 괜찮습니다.
                                                            # self.logger가 print가 될 수도, rclpy 로거가 될 수도 있음
        self.lcd = None
        try:
            self.lcd = LCD()
            self.logger.info("LCD 컨트롤러 초기화 완료.") # .info() 메서드 사용
        except Exception as e:
            self.logger.error(f"LCD 초기화 실패: {e}. LCD 기능이 비활성화됩니다.") # .error() 메서드 사용
            self.lcd = None

    def play_gif(self, gif_path, frame_delay=0.05):
        if not self.lcd:
            self.logger.warn("LCD가 초기화되지 않아 GIF를 재생할 수 없습니다.") # .warn() 메서드 사용
            return

        if not os.path.exists(gif_path):
            self.logger.error(f"GIF 파일 '{gif_path}'을(를) 찾을 수 없습니다.") # .error() 메서드 사용
            return

        try:
            self.logger.info(f"GIF 재생 시작: {gif_path}") # .info() 메서드 사용
            img = Image.open(gif_path)
            for frame in ImageSequence.Iterator(img):
                self.lcd.img_show(frame)
                time.sleep(frame_delay)
            self.logger.info("GIF 재생 종료.") # .info() 메서드 사용
        except Exception as e:
            self.logger.error(f"GIF 재생 중 오류 발생: {e}") # .error() 메서드 사용

    def clear_lcd(self):
        if self.lcd:
            try:
                # ... (LCD 화면 지우는 로직) ...
                self.logger.info("LCD 화면을 지웠습니다.") # .info() 메서드 사용
            except Exception as e:
                self.logger.error(f"LCD 화면 지우기 실패: {e}") # .error() 메서드 사용

    def close(self):
        if self.lcd:
            try:
                self.lcd.close()
                self.logger.info("LCD 컨트롤러 종료 완료.") # .info() 메서드 사용
            except Exception as e:
                self.logger.error(f"LCD 종료 중 오류 발생: {e}") # .error() 메서드 사용
            self.lcd = None