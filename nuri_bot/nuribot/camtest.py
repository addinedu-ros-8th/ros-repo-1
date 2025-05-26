import cv2

try:
    cap = cv2.VideoCapture("/dev/webcam")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) == 27:  # ESC 키 누르면 종료
            break
except Exception as e:
    print(e)
finally:
    cv2.destroyAllWindows()