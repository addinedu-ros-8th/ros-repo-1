
from gtts import gTTS
from playsound import playsound  # pip install playsound

def text2speech(text, output_file="output.mp3"):
    try:
        tts = gTTS(text=text, lang='ko')
        tts.save(output_file)
        print(f"TTS 변환 완료: {output_file}")
        playsound(output_file)
        # 필요하다면 생성된 파일 삭제
        # os.remove(output_file)
    except Exception as e:
        print(f"gTTS 오류 발생: {e}")


if __name__ == "__main__":
    llm_output = "안녕하세요. Google gTTS를 통해 텍스트를 음성으로 변환합니다."
    text2speech(llm_output)

    llm_output_2 = "이것은 gTTS를 사용한 또 다른 음성 출력 예시입니다."
    text2speech(llm_output_2, "output2.mp3")