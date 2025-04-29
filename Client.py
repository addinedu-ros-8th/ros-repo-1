import socket
import cv2
import struct
import Constants as cons
from CameraModule import CameraModule

# UDP socket 생성
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_frame(camera: CameraModule, server_ip, server_port):
    """UDP를 통해 프레임 전송"""
    try:
        frame = camera.read_frame()

        # 프레임 전처리 (크기 조정, 압축 등) - 선택 사항
        frame = cv2.resize(frame, (640, 480))
        encode_param = [cv2.IMWRITE_JPEG_QUALITY, 90]  # 압축률 설정
        result, frame_encoded = cv2.imencode('.jpg', frame, encode_param)
        if not result:
            raise ValueError("Frame encoding failed")
        frame_bytes = frame_encoded.tobytes()

        # 데이터 크기 전송 (AI 서버에서 수신 준비)
        data_size = len(frame_bytes)
        sock.sendto(struct.pack("<I", data_size), (server_ip, server_port))

        # 프레임 데이터 전송
        sock.sendto(frame_bytes, (server_ip, server_port))

        print(f"Sent frame (size: {len(frame_bytes)} bytes)")

    except ValueError as ve:
        print(f"ValueError: {ve}")
    except socket.error as se:
        print(f"Socket error: {se}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        camera.release()
        sock.close()  # 소켓 닫기


if __name__ == "__main__":
    camera = CameraModule()
    try:
        send_frame(camera, cons.SERVER_IP, cons.UDP_PORT)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        camera.release()
        cv2.destroyAllWindows()