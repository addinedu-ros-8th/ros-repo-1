import cv2


class CameraModule:
    def __init__ (self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened(): raise ValueError("Could not open camera")
    
    def read_frame(self):
        ret, frame = self.cap.read()
        if not ret: raise ValueError("Could not open camera")
        return frame 
    
    def release(self):
        self.cap.release()
    

if __name__ == "__main__":
    try : 
        camera = CameraModule()
        while True:
            frame = camera.read_frame()
            cv2.imshow("Camera Preview", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except ValueError as e:
        print(f"Error: {e}")
    finally:
        if 'cameara' in locals():
            camera.release()
        cv2.destroyAllWindows() 