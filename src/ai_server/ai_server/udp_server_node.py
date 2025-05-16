
# ROS reated libraries
import rclpy
from rclpy.node import Node

# vision related libraries
import socket
import threading
import numpy as np
import struct  # 추가: 데이터 크기 처리를 위해 
import cv2
import torch 
from ultralytics import YOLO

# Global Initialization
from emergency.srv import EmergencyReport   # service 
from config.config_node import UserConfig
config = UserConfig()

class UdpImageReceiver(Node):
    def __init__(self):
        super().__init__('udp_image_receiver')
        self.udp_address = ('0.0.0.0', 5000)  # 수신할 주소 및 포트 (모든 인터페이스에서 수신)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.udp_address)

        self.timer = self.create_timer(0.033, self.receive_frame)  # 30Hz로 수신 처리
        # threading.Thread(target=self.receive_frame, daemon=True).start()

        # init service client 
        self.emergency_client = self.create_client(EmergencyReport, 'report_emergency')
        while not self.emergency_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')


        self.target_track_id = None
        self.target_identified = False
        self.initial_scan_duration = 5.0
        self.scan_start_time = self.get_clock().now().nanoseconds * 1e-9
        self.target_features = None
        self.reid_similarity_threshold = 0.8 # Adjust as needed
    def receive_frame(self):
        """
            Decode the image data
        """
        # while True:
        try:
            size_bytes, _ = self.sock.recvfrom(65536)
            img_size = struct.unpack(">I", size_bytes[:4])[0]

            img_bytes = size_bytes[4:  4 + img_size]
            img_np = np.frombuffer(img_bytes, dtype=np.uint8)
            frame = cv2.imdecode(img_np, cv2.IMREAD_COLOR)

            # robot_id 수신 (1바이트, unsigned int8)
            robot_id = struct.unpack(">B", size_bytes[4 + img_size:4+img_size+1])[0]
            # command 수신 (1바이트, unsigned int8)
            command = struct.unpack(">B", size_bytes[4 +  img_size + 1: 4+img_size+2])[0]

            if frame is not None:
                # make 


                processed_frame = self.which_command(command, frame.copy(), robot_id)
                cv2.imshow(f'Received Frame (UDP) - Robot: {robot_id}', processed_frame)
                cv2.waitKey(1)
                # self.detect_older_person(frame,robot_id)
                # self.detect_emergency(frame, robot_id) 
            else:
                self.logger.warn('Failed to decode image')

        except socket.error as e:
            self.logger.error(f'Socket error: {e}')
        except struct.error as e:
            self.logger.error(f'Struct error: {e}')
        except Exception as e:
            self.logger.error(f'Error receiving frame: {e}')

    def which_command(self, command, frame , robot_id):
        """
            command에 따라 처리하는 함수 
        """
        # self.get_logger().info(f"Executing command: {command}")
        if command == 1:
            return self.detect_older_person(frame, robot_id)
        elif command == 2:
            self.detect_emergency(frame, robot_id)
            return frame 
        else : 
            return frame 
             
    def detect_older_person(self, frame, robot_id):        
        """
            노인 인식 시, 깊이 추정
        """
        model = YOLO('yolov8n.pt')
        # model = YOLO('yolo11n-cls.pt') # higher accuracy

        # 0: person class id -> detect only person  
        # borsort: multi-object tracking algorithm
        results = model.track(frame, conf=config.confidence_threshold,
                               persist=True, classes=[0], 
                               tracker='botsort.yaml', show=False,verbose=False) 

        target_result = None
        if isinstance(results, list):
            for result in results:
                if self.identify_target_person(result) is not None:
                    target_result = result
                    # break
        elif hasattr(results, 'boxes'):
            if self.identify_target_person(results) is not None:
                target_result = results

        if target_result:
            target_track_id = self.identify_target_person(target_result)
            if target_track_id is not None and \
                hasattr(target_result, 'boxes') and\
                hasattr(target_result.boxes, 'id') and\
                    target_result.boxes.id is not None:
                
                tracked_ids = target_result.boxes.id.cpu().numpy().astype(int)
                xyxy_coords = target_result.boxes.xyxy.cpu().numpy().astype(int)
                if hasattr(target_result, 'masks') and hasattr(target_result.masks, 'data'):
                    masks_data = target_result.masks.data.cpu().numpy()

                for i, track_id in enumerate(tracked_ids):
                    if track_id == target_track_id:
                        x1, y1, x2, y2 = xyxy_coords[i]
                        frame = self._visualize_tracking(frame, target_result, i)
                        
                        # distance = self._estimate_depth(frame, masks_data[i] if masks_data is not None and i < len(masks_data) else None)
                        # if distance is not None:
                        #     self.get_logger().info(f"Robot {robot_id}: Estimated distance to older person (ID: {track_id}): {distance:.2f}")
                        #     self.control_robot(distance, (x1, y1, x2, y2), robot_id)
                        # else:
                        #     self.get_logger().warn(f"Robot {robot_id}: Could not estimate depth for the older person.")
                        # break

        return frame
    
    def identify_target_person(self, result):
        """
            카메라 setup시 첫번째 사람을 어르신으로 인식 
        """
        if hasattr(result, 'boxes') and hasattr(result.boxes, 'id') and result.boxes.id is not None:
            track_ids = result.boxes.id.cpu().numpy().astype(int)
            if len(track_ids) > 0:
                return track_ids[0]
        return None
   

    def _visualize_tracking(self, frame, result, index):
        if hasattr(result, 'boxes') and hasattr(result.boxes, 'id') and result.boxes.id is not None:
            tracked_ids = result.boxes.id.cpu().numpy().astype(int)
            xyxy_coords = result.boxes.xyxy.cpu().numpy().astype(int)
            x1, y1, x2, y2 = xyxy_coords[index]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"ID: {tracked_ids[index]}", (x1, y1 - 10),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            if hasattr(result, 'masks') and hasattr(result.masks, 'data'):
                masks_data = result.masks.data.cpu().numpy()
                if index < len(masks_data):
                    mask = masks_data[index]
                    mask_resized = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                    binary_mask = (mask_resized > 0.5).astype(np.uint8) * 255
                    colored_mask = np.zeros_like(frame, dtype=np.uint8)
                    colored_mask[:, :, 1] = binary_mask
                    frame = cv2.addWeighted(frame, 0.7, colored_mask, 0.3, 0)
        return frame

        
    def detect_emergency(self, frame, robot_id): 
        """
            사람 or 불 인지 함수 
        """
        model = YOLO('/home/addinedu/dev_ws/project/ros-repo-1/runs/detect/train4/weights/best.pt')  
        results = model(frame)
        
        fire_detected = False
        fall_detected = False

        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                class_id = int(box.cls)
                confidence = float(box.conf)
                class_name = model.names[class_id]

                if confidence > config.confidence_threshold:  # 설정 파일에서 신뢰도 임계값 사용
                    if class_name == 'Fire':
                        fire_detected = True
                        self.logger.warn(f"Robot {robot_id}: Fire detected with confidence {confidence:.2f}")
                        # 화재 감지에 따른 추가 로직 (예: send_emergency 호출)
                        self.send_emergency(robot_id, 'Fire')
                    elif class_name == 'Fall':
                        fall_detected = True
                        self.logger.info(f"Robot {robot_id}: Fall detected with confidence {confidence:.2f}")
                        self.send_emergency(robot_id, 'Fall')
    

     
    def estimate_depth_for_mask(self, frame, mask):
        """
            mask에 해당하는 영역의 깊이를 추정하는 함수
        """
       

    def send_emergency (self, robot_id, emergency_type):
        """
            긴급 상황 발생 시, tcp_server_node에 알림을 전송합니다.    
        """
        # Request msg
        req = EmergencyReport.Request()  
        req.robot_id = robot_id
        req.emergency_type = emergency_type
        # Call service 
        self.emergency_client.call_async(req) # async; 응답을 기다리는 동안 다른 작업 수행 가능 ! 
        self.logger.info(f'Sent emergency to TCP server via service: Robot {robot_id} - {emergency_type}')

    def destroy_node(self):
        self.sock.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpImageReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()