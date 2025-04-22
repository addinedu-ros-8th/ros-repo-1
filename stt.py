import os
from pydub import AudioSegment
import speech_recognition as sr

def convert_to_wav(input_path, output_path=None):
    """
    오디오 파일을 WAV로 변환하는 함수

    :param input_path: 변환할 원본 오디오 파일 경로
    :param output_path: 변환된 WAV 파일 경로 (기본적으로 동일한 폴더에 저장)
    :return: 변환된 WAV 파일 경로
    """
    if not os.path.exists(input_path):
        raise FileNotFoundError(f"파일을 찾을 수 없습니다. {input_path}")

    # 출력 경로 설정
    if output_path is None:
        output_path = os.path.splitext(input_path)[0] + ".wav"

    try:
        # 오디오 파일 로드 및 WAV로 변환
        audio = AudioSegment.from_file(input_path)
        audio.export(output_path, format="wav")
        print(f"변환 완료: {output_path}")
        return output_path
    except Exception as e:
        print(f"오디오 변환 중 오류 발생: {e}")
        return None

def speech_to_text_google(audio_file):
    """
    Google Speech Recognition API를 사용하여 음성을 텍스트로 변환하는 함수
    (인터넷 연결 필요)

    :param audio_file: 변환할 WAV 파일 경로
    :return: 인식된 텍스트 (성공 시), 오류 메시지 (실패 시)
    """
    recognizer = sr.Recognizer()

    if not os.path.exists(audio_file):
        return "파일이 존재하지 않습니다."

    try:
        with sr.AudioFile(audio_file) as source:
            audio_data = recognizer.record(source)  # 전체 오디오 파일 읽기

        text = recognizer.recognize_google(audio_data, language="ko-KR")
        return text
    except sr.UnknownValueError:
        return "음성을 인식할 수 없습니다."
    except sr.RequestError as e:
        return f"Google Speech Recognition API 요청 오류: {e}"

# 음성 파일 입력 (MP4 -> WAV 변환)
audio_file_path = "./salute.mp4"
wav_file_path = convert_to_wav(audio_file_path)

if wav_file_path:
    # 변환된 WAV 파일로 Google Speech Recognition을 사용하여 텍스트 추출
    recognized_text = speech_to_text_google(wav_file_path)
    print(f"Google Speech Recognition으로 인식된 텍스트: {recognized_text}")
else:
    print("WAV 파일 변환에 실패했습니다.")