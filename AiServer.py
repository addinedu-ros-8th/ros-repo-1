import socket
import cv2
import numpy as np
import struct
from ultralytics import YOLO
import Constants as cons


model = YOLO("yolov10s.pt")  # Load your YOLO model
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((cons.SERVER_IP, cons.UDP_PORT))


def receive_frame(sock: socket.socket):
    """Receive frame data over UDP."""

    try:
        size_bytes = sock.recv(4)  # Initial 4 bytes for frame size
        size = struct.unpack("<I", size_bytes)[0]
        frame_bytes = b""
        while len(frame_bytes) < size:
            remaining_bytes = size - len(frame_bytes)
            frame_bytes += sock.recv(remaining_bytes)
        frame_np = np.frombuffer(frame_bytes, dtype=np.uint8)
        frame = cv2.imdecode(frame_np, cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError("Failed to decode frame")
        return frame
    except struct.error as e:
        print(f"Struct error: {e}")
        return None
    except ValueError as ve:
        print(f"ValueError: {ve}")
        return None
    except socket.error as se:
        print(f"Socket error: {se}")
        return None
    except Exception as e:
        print(f"Error in receive_frame: {e}")
        return None


def process_frame(frame, model: YOLO):
    """Process the frame with YOLO for object detection."""

    results = model.predict(frame, verbose=False)  # Perform YOLO inference
    # Process the results (e.g., draw bounding boxes)
    for result in results:
        for box in result.boxes:
            xyxy = box.xyxy.int().tolist()
            cv2.rectangle(frame, (xyxy[0][0], xyxy[0][1]), (xyxy[0][2], xyxy[0][3]), (0, 255, 0), 2)  # Green box
    return frame


def main():
    try:
        while True:
            frame = receive_frame(sock)
            if frame is None:
                print("Error receiving or decoding frame, continuing...")
                continue  # Skip processing if no frame

            processed_frame = process_frame(frame.copy(), model)  # Process the frame (copy to avoid modifying original)
            cv2.imshow("YOLO Detections", processed_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break  # Exit on 'q' key

    except KeyboardInterrupt:
        print("Server interrupted, closing...")
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()