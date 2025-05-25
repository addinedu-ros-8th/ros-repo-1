# ROS reated libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# vision related libraries
import socket
import threading
import numpy as np
import struct  # 추가: 데이터 크기 처리를 위해 
import cv2
import torch
from ultralytics import YOLO

# Import our module 
from ai_server.detection_model import ObjectDetector
from ai_server.depth_model import DepthEstimator
from services.srv import Emergency, Distance    # service 
# 기존 imports 아래에 추가
from collections import deque

# Global Initialization
from config.config_node import UserConfig
config = UserConfig()
"""
Person Re-identification Module (개선된 버전)
프레임을 벗어났다가 다시 들어오는 사람을 같은 ID로 추적하는 모듈
"""

import cv2
import numpy as np
from collections import deque, defaultdict
import time
class OptimizedPersonReID:
    """
    ID=1 전용 최적화된 ReID 클래스
    성능 향상을 위해 ID=1인 사람만 추적하고 재식별
    """
    
    def __init__(self, max_disappeared_frames=90, feature_history_size=10, similarity_threshold=0.5):
        """
        Args:
            max_disappeared_frames (int): 사라진 후 몇 프레임까지 기억할지
            feature_history_size (int): ID=1의 특징을 몇 개까지 저장할지
            similarity_threshold (float): 재식별 판정 임계값 (0~1)
        """
        self.max_disappeared_frames = max_disappeared_frames
        self.feature_history_size = feature_history_size
        self.similarity_threshold = similarity_threshold
        
        # ID=1 전용 추적 정보
        self.id1_active = None  # {'features': deque, 'last_seen': frame_count, 'bbox': last_bbox}
        self.id1_disappeared = None  # {'features': deque, 'disappeared_frame': frame_count}
        
        self.frame_count = 0
        self.debug_mode = True
        
    def extract_simple_features(self, frame, bbox):
        """
        간단한 특징 추출 (성능 최적화)
        """
        x1, y1, x2, y2 = [int(coord) for coord in bbox]
        
        # 바운딩 박스 영역 추출
        person_region = frame[y1:y2, x1:x2]
        
        if person_region.size == 0:
            return np.zeros(64)  # 더 작은 특징 벡터
        
        # 크기 정규화 (작은 크기)
        person_region = cv2.resize(person_region, (32, 64))
        
        # 간단한 색상 특징만 추출
        # 상체/하체 평균 색상
        height, width = person_region.shape[:2]
        upper_body = person_region[:int(height*0.6), :]
        lower_body = person_region[int(height*0.4):, :]
        
        # BGR 평균 색상
        upper_mean = np.mean(upper_body.reshape(-1, 3), axis=0) / 255.0
        lower_mean = np.mean(lower_body.reshape(-1, 3), axis=0) / 255.0
        
        # 크기 특징
        size_features = np.array([width/32.0, height/64.0, width/height])
        
        # 위치 특징
        center_x = (x1 + x2) / 2.0 / frame.shape[1]
        center_y = (y1 + y2) / 2.0 / frame.shape[0]
        position_features = np.array([center_x, center_y])
        
        # 간단한 특징 결합 (총 11개)
        features = np.concatenate([
            upper_mean,       # 3 features
            lower_mean,       # 3 features
            size_features,    # 3 features
            position_features # 2 features
        ])
        
        # 64차원으로 패딩
        padded_features = np.zeros(64)
        padded_features[:len(features)] = features
        
        return padded_features
    
    def calculate_similarity(self, features1, features2):
        """
        간단한 유사도 계산
        """
        # 코사인 유사도만 사용 (빠름)
        norm1 = np.linalg.norm(features1) + 1e-8
        norm2 = np.linalg.norm(features2) + 1e-8
        
        cosine_sim = np.dot(features1, features2) / (norm1 * norm2)
        return max(0, min(1, cosine_sim))
    
    def is_id1_match(self, new_features):
        """
        새로운 특징이 ID=1과 매칭되는지 확인
        """
        if self.id1_active is not None:
            # 활성 ID=1과 비교
            similarities = []
            for stored_features in self.id1_active['features']:
                sim = self.calculate_similarity(new_features, stored_features)
                similarities.append(sim)
            
            if similarities:
                max_sim = max(similarities)
                if max_sim >= self.similarity_threshold:
                    if self.debug_mode:
                        print(f"[ReID] Active ID=1 matched (similarity: {max_sim:.3f})")
                    return True, max_sim
        
        if self.id1_disappeared is not None:
            # 사라진 ID=1과 비교 (재식별)
            similarities = []
            for stored_features in self.id1_disappeared['features']:
                sim = self.calculate_similarity(new_features, stored_features)
                similarities.append(sim)
            
            if similarities:
                max_sim = max(similarities)
                if max_sim >= self.similarity_threshold:
                    if self.debug_mode:
                        print(f"[ReID] ⭐ ID=1 re-identified! (similarity: {max_sim:.3f})")
                    # 사라진 것을 다시 활성화
                    self.id1_active = self.id1_disappeared.copy()
                    self.id1_disappeared = None
                    return True, max_sim
        
        return False, 0.0
    
    def update_tracking(self, frame, detections_list):
        """
        ID=1만 추적하는 최적화된 업데이트
        """
        self.frame_count += 1
        
        if self.debug_mode:
            print(f"\n[ReID DEBUG] Frame {self.frame_count}: Looking for ID=1...")
        
        # ID=1 검출 여부 확인
        id1_detected = False
        id1_detection = None
        
        # ID=1만 찾기
        for detection in detections_list:
            bbox, score, class_id, original_id = detection
            
            # 사람 클래스이고 원본 ID가 1인 경우
            if class_id == 0 and original_id == 1:
                id1_detected = True
                id1_detection = detection
                
                # 특징 추출
                features = self.extract_simple_features(frame, bbox)
                
                # ID=1 정보 업데이트
                if self.id1_active is None:
                    self.id1_active = {
                        'features': deque(maxlen=self.feature_history_size),
                        'last_seen': self.frame_count,
                        'bbox': bbox
                    }
                    if self.debug_mode:
                        print(f"[ReID] ID=1 newly detected and tracked")
                
                # 특징 추가
                self.id1_active['features'].append(features)
                self.id1_active['last_seen'] = self.frame_count
                self.id1_active['bbox'] = bbox
                break
        
        # ID=1이 검출되지 않은 경우
        if not id1_detected:
            # 다른 사람들 중에서 ID=1과 유사한 사람 찾기 (재식별)
            for detection in detections_list:
                bbox, score, class_id, original_id = detection
                
                if class_id == 0:  # 사람 클래스
                    features = self.extract_simple_features(frame, bbox)
                    is_match, similarity = self.is_id1_match(features)
                    
                    if is_match:
                        # ID=1으로 재할당
                        id1_detected = True
                        id1_detection = [bbox, score, class_id, 1]  # ID를 1로 변경
                        
                        # 특징 업데이트
                        self.id1_active['features'].append(features)
                        self.id1_active['last_seen'] = self.frame_count
                        self.id1_active['bbox'] = bbox
                        break
        
        # ID=1이 여전히 없으면 사라진 것으로 처리
        if not id1_detected and self.id1_active is not None:
            frames_since_seen = self.frame_count - self.id1_active['last_seen']
            if frames_since_seen > 3:  # 3프레임 연속으로 안 보이면
                self.id1_disappeared = {
                    'features': self.id1_active['features'],
                    'disappeared_frame': self.frame_count
                }
                self.id1_active = None
                if self.debug_mode:
                    print(f"[ReID] ID=1 marked as disappeared")
        
        # 너무 오래된 사라진 ID=1 삭제
        if self.id1_disappeared is not None:
            if self.frame_count - self.id1_disappeared['disappeared_frame'] > self.max_disappeared_frames:
                self.id1_disappeared = None
                if self.debug_mode:
                    print(f"[ReID] ID=1 permanently removed from tracking")
        
        # 결과 반환: ID=1이 있으면 해당 detection, 없으면 None
        return id1_detection
    
    def get_stats(self):
        """
        통계 정보 반환
        """
        return {
            'id1_active': self.id1_active is not None,
            'id1_disappeared': self.id1_disappeared is not None,
            'frame_count': self.frame_count,
            'similarity_threshold': self.similarity_threshold
        }
    

    
class UdpImageReceiver(Node):
    """
        Nuri(pinky)로 부터 frame을 udp로 송신받고, tcp 노드와 main server와 통신하는 ai server 
    """
    def __init__(self):
        super().__init__('udp_image_receiver')
        self.udp_address = ('0.0.0.0', 5000)  # 수신할 주소 및 포트 (모든 인터페이스에서 수신)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.udp_address)

        self.timer = self.create_timer(0.033, self.receive_frame)  # 30Hz로 수신 처리

        # init service client 
        self.__srv_emergency_client = self.create_client(Emergency, 'report_emergency')
        self.__srv_distance_client = self.create_client(Distance, '/report_distance')


        # init models
        self.get_logger().info("init models...")
        try:
            self.detector = ObjectDetector(
                model_size= config.yolo_model_size,
                conf_thres= config.conf_threshold,
                iou_thres= config.iou_threshold,
                classes= config.classes,
                ros_logger= self.get_logger()     
            )
        except Exception as e:
            self.get_logger().info(f"Error initializing object detector: {e}")
            self.get_logger().info("Falling back to CPU for object detection")
            self.detector = ObjectDetector(
                model_size=config.yolo_model_size,
                conf_thres=config.conf_threshold,
                iou_thres=config.iou_threshold,
                classes=config.classes,
                device='cpu',
                ros_logger= self.get_logger()
            )

        try:
            self.depth_estimator = DepthEstimator(
                model_size = config.depth_model_size, 
                
            ) 
        except Exception as e:
            print(f"Error init depth estimator: {e}") 
            print("Falling back to CPU for depth estimation")
            self.depth_estimator = DepthEstimator(
                model_size = config.depth_model_size, 
                device = 'cpu'  # force to use cpu 
            )
        self.target_track_id = None
        self.last_target_update_time = self.get_clock().now().nanoseconds * 1e-9 # For re-selection interval
        self.target_reselection_interval = 2.0 # How often to re-evaluate the closest person (in seconds)

        # Debug 
        self.__pub_for_closefar = self.create_publisher(       
            String, "/report_closefar", 10)
        
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

            robot_id = struct.unpack(">B", size_bytes[4 + img_size:4+img_size+1])[0]
            command = struct.unpack(">B", size_bytes[4 +  img_size + 1: 4+img_size+2])[0]

            if frame is not None:                
                processed_frame = self.which_command(command, frame, robot_id)
                # processed_frame = self.detect_emergency(frame, robot_id)
                
                cv2.imshow(f'Received Frame (UDP) - Nuri: {robot_id}', processed_frame)
                cv2.waitKey(1)
            else:
                self._logger.warn('Failed to decode image')

        except socket.error as e:
            self._logger.error(f'Socket error: {e}')
            
        except struct.error as e:
            self._logger.error(f'Struct error: {e}')
        except Exception as e:
            self._logger.error(f'Error receiving frame: {e}')

    def which_command(self, command, frame , robot_id):
        """
            command에 따라 처리하는 함수 
        """
        if command == 1:
            return self.detect_older_person(frame, robot_id)
        else :
            return self.detect_emergency(frame, robot_id)
         
    def detect_older_person(self, frame, robot_id):        
        """
            OptimizedPersonReID 전용 버전 (ID=1만 추적)
        """
        if self.detector is None:
            self.get_logger().error("ObjectDetector is not initialized. Cannot perform detection.")
            return frame
        
        # OptimizedPersonReID 시스템 초기화 (첫 실행시에만)
        if not hasattr(self, 'reid_system'):
            self.reid_system = OptimizedPersonReID(
                max_disappeared_frames=90,  
                feature_history_size=10,    
                similarity_threshold=0.5    
            )
            self.get_logger().info("Optimized ReID system initialized (ID=1 only)")
        
        detection_frame = frame.copy()
        distance_state = None  # 거리 상태 변수 추가
        try:
            # 1. 순수 검출
            detections_list = self.detector.detect_only(detection_frame, track=True)
            
            # 디버깅: 원본 검출 결과 로깅
            person_detections = [det for det in detections_list if det[2] == 0]  # person class only
            if person_detections:
                person_ids = [det[3] for det in person_detections]
                self.get_logger().info(f"[DEBUG] YOLO detected persons with IDs: {person_ids}")

            # 2. OptimizedPersonReID로 ID=1만 추적
            id1_detection = self.reid_system.update_tracking(frame, detections_list)
            
            # 3. 깊이 맵 (ID=1이 있을 때만)
            full_depth_map, depth_colored_viz = None, None
            target_norm_depth = None
            
            if id1_detection is not None and self.depth_estimator is not None:
                bbox = id1_detection[0]
                full_depth_map, depth_colored_viz = self.estimate_depth_value(frame)
                
                if full_depth_map is not None:
                    target_norm_depth = self.depth_estimator.get_depth_in_region(
                        full_depth_map, bbox, method='median')
                    self.get_logger().info(f"[TARGET] ID=1 found with depth {target_norm_depth:.3f}")

            # 4. 깔끔한 그리기
            
            # 4-1. 다른 사람들 (회색으로 간단하게)
            for det in detections_list:
                bbox, score, class_id, obj_id = det
                if class_id == 0 and obj_id != 1:  # person but not ID=1
                    x1, y1, x2, y2 = [int(coord) for coord in bbox]
                    
                    # 회색 박스
                    cv2.rectangle(detection_frame, (x1, y1), (x2, y2), (128, 128, 128), 1)
                    
                    # 간단한 ID만
                    if obj_id is not None:
                        cv2.putText(detection_frame, f"{obj_id}", (x1, y1-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)

            # 4-2. ID=1 (빨간색으로 강조)
            if id1_detection is not None:
                self.target_track_id = 1
                target_bbox_coords = id1_detection[0]
                x1, y1, x2, y2 = [int(coord) for coord in target_bbox_coords]
                
                # 빨간색 두꺼운 박스
                cv2.rectangle(detection_frame, (x1, y1), (x2, y2), (0, 0, 255), 4)
                
                # ID=1 trajectory 그리기
                if 1 in self.detector.tracking_trajectories:
                    trajectory = self.detector.tracking_trajectories[1]
                    for i in range(1, len(trajectory)):
                        thickness = int(2 * (i / len(trajectory)) + 1)
                        cv2.line(detection_frame, 
                                (int(trajectory[i-1][0]), int(trajectory[i-1][1])), 
                                (int(trajectory[i][0]), int(trajectory[i][1])), 
                                (0, 0, 255), thickness)
                
                # ReID 라벨만 간단하게
                reid_label = "ReID:1"
                
                # 텍스트 배경
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.7
                font_thickness = 2
                
                label_size = cv2.getTextSize(reid_label, font, font_scale, font_thickness)[0]
                cv2.rectangle(detection_frame, 
                            (x1-2, y1-30), 
                            (x1 + label_size[0] + 4, y1-5), 
                            (0, 0, 0), -1)  # 검은색 배경
                
                # ReID 텍스트 (노란색)
                cv2.putText(detection_frame, reid_label, (x1, y1 - 12),
                        font, font_scale, (0, 255, 255), font_thickness)
                
                # 로봇 제어 및 거리 상태 얻기
                if target_norm_depth is not None:
                    distance_state = self.control_robot(target_norm_depth, target_bbox_coords, robot_id)
                
                # 깊이 맵 표시
                if depth_colored_viz is not None:
                    self._draw_target_depth_on_colored_map(
                        depth_colored_viz, target_bbox_coords, 1, target_norm_depth or 0.0)
                    # cv2.imshow(f'Depth Map - Nuri: {robot_id}', depth_colored_viz)
                    # cv2.waitKey(1)
            else:
                # ID=1 없음
                self.target_track_id = None

            # 5. 상단 상태 표시
            reid_stats = self.reid_system.get_stats()
            
            if id1_detection is not None:
                if target_norm_depth is not None:
                    status_text = f"Tracking ReID:1 | Depth:{target_norm_depth:.2f}"
                else:
                    status_text = f"Tracking ReID:1 | No Depth"
                text_color = (0, 255, 255)  # 노란색
            else:
                reid_status = "Active" if reid_stats['id1_active'] else ("Disappeared" if reid_stats['id1_disappeared'] else "None")
                status_text = f"No ID=1 | Status:{reid_status} | Frame:{reid_stats['frame_count']}"
                text_color = (128, 128, 128)  # 회색

            # 상태 텍스트 배경
            text_size = cv2.getTextSize(status_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            cv2.rectangle(detection_frame, (5, 5), (text_size[0] + 15, 35), (0, 0, 0), -1)
            cv2.putText(detection_frame, status_text, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)

            # 6. 거리 상태 표시 (새로 추가)
            if distance_state is not None:
                # 거리 상태 텍스트 설정
                distance_text = f"Distance: {distance_state.upper()}"
                
                # 거리 상태에 따른 색상 설정
                if distance_state == "close":
                    distance_color = (0, 0, 255)  # 빨간색 (가까움)
                else:  # "far"
                    distance_color = (0, 255, 0)  # 초록색 (멀음)
                
                # 텍스트 크기 계산
                distance_font = cv2.FONT_HERSHEY_SIMPLEX
                distance_font_scale = 1.0
                distance_font_thickness = 3
                distance_text_size = cv2.getTextSize(distance_text, distance_font, distance_font_scale, distance_font_thickness)[0]
                
                # 텍스트 위치 (화면 상단 중앙)
                text_x = (detection_frame.shape[1] - distance_text_size[0]) // 2
                text_y = 60
                
                # 배경 그리기
                cv2.rectangle(detection_frame, 
                            (text_x - 10, text_y - distance_text_size[1] - 10), 
                            (text_x + distance_text_size[0] + 10, text_y + 10), 
                            (0, 0, 0), -1)  # 검은색 배경
                
                # 텍스트 그리기
                cv2.putText(detection_frame, distance_text, (text_x, text_y),
                        distance_font, distance_font_scale, distance_color, distance_font_thickness)

        except Exception as e:
            self.get_logger().error(f"Error during detection or ReID processing: {e}")
            cv2.putText(detection_frame, "Processing Error", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        return detection_frame
    
    def estimate_depth_value(self, frame):
        """
            mask에 해당하는 영역의 깊이를 추정하는 함수
        """
        try:
            depth_map = self.depth_estimator.estimate_depth(frame)
            depth_colored = self.depth_estimator.colorize_depth(depth_map)
            
        except Exception as e:
                print(f"Error during depth estimation: {e}")
                # Create a dummy depth map
                depth_map = np.zeros((640, 480), dtype=np.float32)
                depth_colored = np.zeros((640, 480, 3), dtype=np.uint8)
                cv2.putText(depth_colored, "Depth Error", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        return depth_map, depth_colored 
    
    def _draw_target_depth_on_colored_map(self, depth_colored_map, person_bbox, person_id, norm_depth_value):
        x1, y1, x2, y2 = [int(coord) for coord in person_bbox]
        
        # 텍스트 준비
        text = f"ID:{person_id} Norm:{norm_depth_value:.2f}"
        
        # 폰트 설정
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.8  # 더 큰 폰트 크기
        font_thickness = 2
        
        # 텍스트 크기 계산
        text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
        text_width, text_height = text_size
        
        # 바운딩 박스 중앙 위치 계산
        center_x = x1 + (x2 - x1) // 2
        center_y = y1 + (y2 - y1) // 2
        
        # 텍스트를 위한 위치 계산
        text_x = center_x - text_width // 2
        text_y = center_y + text_height // 2
        
        # 텍스트 배경 그리기 (가독성 향상을 위해)
        padding = 10
        cv2.rectangle(depth_colored_map, 
                    (text_x - padding, text_y - text_height - padding), 
                    (text_x + text_width + padding, text_y + padding), 
                    (0, 0, 0), -1)  # 검은색 배경
        
        # 텍스트 그리기
        cv2.putText(depth_colored_map, text, (text_x, text_y),
                    font, font_scale, (255, 255, 255), font_thickness)  # 흰색 텍스트
        
        # 바운딩 박스 그리기
        cv2.rectangle(depth_colored_map, (x1, y1), (x2, y2), (255, 255, 255), 2)

    def control_robot(self, norm_depth, bbox_coords, robot_id):
        """
        Args:
            norm_depth (float): 정규화된 깊이 값 (0~1)
            bbox_coords (list): 바운딩 박스 좌표 [x1, y1, x2, y2]
            robot_id (int): 로봇 ID
        """

        
        close_threshold = 0.65  # 이 값 이상이면 "close"
        far_threshold = 0.4    # 이 값 이하면 "far"
        state = None 
        if norm_depth > close_threshold :        
            state = "close"
        else:
            state = "far" 

        self.report_distance(robot_id, state)       
        self.get_logger().info(f"Robot {robot_id}: Distance state changed to  {state} (depth: {norm_depth:.2f})")
        
        return state

    def report_distance(self, robot_id, distance_state):
        """
            거리 상태를 TCP 노드에 보고하는 함수 

            Args:
                robot_id (int): 로봇 ID 
                distance_state(str): 거리 상태
        """

        # req = Distance.Request()  
        # req.robot_id = robot_id
        # req.distance_type = distance_state
        # # Call service 
        # self.__srv_distance_client.call_async(req) # async; 응답을 기다리는 동안 다른 작업 수행 가능 ! 

        # Debug 
        msg = String()
        msg.data = distance_state
        self.__pub_for_closefar.publish(msg)
        

    # ================================ Emergency =================================== 
    def detect_emergency(self, frame, robot_id): 
        """
        사람 or 불 인지 함수 
        """
        model = YOLO('/home/addinedu/dev_ws/project/ros-repo-1/runs/detect/train4/weights/best.pt')  
        results = model(frame, verbose=False)
        
        fire_detected = False
        fall_detected = False
        
        # 결과를 시각화할 프레임 복사
        visualization_frame = frame.copy()
        
        # 디버깅: 감지 결과 로깅
        self.get_logger().info(f"Detection results: {len(results)} items found")
        
        for i, result in enumerate(results):
            boxes = result.boxes
            self.get_logger().info(f"Result {i}: found {len(boxes)} boxes")
            
            for j, box in enumerate(boxes):
                try:
                    class_id = int(box.cls[0]) if isinstance(box.cls, torch.Tensor) else int(box.cls)
                    confidence = float(box.conf[0]) if isinstance(box.conf, torch.Tensor) else float(box.conf)
                    class_name = model.names[class_id]
                    
                    self.get_logger().info(f"Box {j}: class={class_name}, conf={confidence:.2f}")
                    
                    # 감지된 모든 객체에 대해 바운딩 박스 그리기 (임계값 무시)
                    try:
                        # YOLOv8에서 바운딩 박스 좌표 가져오기
                        if hasattr(box, 'xyxy'):
                            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                        elif hasattr(box, 'xywh'):
                            x, y, w, h = map(int, box.xywh[0].cpu().numpy())
                            x1, y1 = int(x - w/2), int(y - h/2)
                            x2, y2 = int(x + w/2), int(y + h/2)
                        else:
                            # 직접 YOLOv8 결과에서 좌표 추출
                            # boxes.data 형식: [x1, y1, x2, y2, confidence, class]
                            x1, y1, x2, y2 = map(int, box.data[0][:4].cpu().numpy())
                        
                        # 감지된 바운딩 박스 좌표 로그 출력
                        self.get_logger().info(f"Drawing box at ({x1}, {y1}), ({x2}, {y2})")
                        
                        # 클래스에 따라 다른 색상으로 박스 그리기
                        if class_name == 'Fire':
                            fire_detected = True
                            cv2.rectangle(visualization_frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
                            cv2.putText(visualization_frame, f"Fire", 
                                    (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                            
                            self.get_logger().warn(f"Robot {robot_id}: Fire detected with confidence {confidence:.2f}")
                            if confidence > config.confidence_threshold:
                                self.send_emergency(robot_id, 'Fire')
                            
                        elif class_name == 'Fall':
                            fall_detected = True
                            cv2.rectangle(visualization_frame, (x1, y1), (x2, y2), (0, 165, 255), 3)
                            cv2.putText(visualization_frame, f"Fall", 
                                    (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
                            
                            self.get_logger().info(f"Robot {robot_id}: Fall detected with confidence {confidence:.2f}")
                            if confidence > config.confidence_threshold:
                                self.send_emergency(robot_id, 'Fall')
                    
                    except Exception as e:
                        self.get_logger().error(f"Error drawing bounding box: {str(e)}")
                
                except Exception as e:
                    self.get_logger().error(f"Error processing detection box: {str(e)}")
        
        # 화면 상단에 긴급 상황 요약 표시
        if fire_detected or fall_detected:
            status_text = "EMERGENCY: "
            if fire_detected:
                status_text += "FIRE "
            if fall_detected:
                status_text += "PERSON FALLEN"
                
            cv2.putText(visualization_frame, status_text, 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        
        # 디버깅: 프레임 저장하여 확인 (선택 사항)
        cv2.imwrite(f'/tmp/emergency_frame_{robot_id}.jpg', visualization_frame)

        # 처리된 프레임 반환
        return visualization_frame
    
    def send_emergency (self, robot_id, emergency_type):
        """
            긴급 상황 발생 시, tcp_server_node에 알림을 전송합니다.    
        """
        # Request msg
        req = Emergency.Request()  
        req.robot_id = robot_id
        req.emergency_type = emergency_type
        # Call service 
        self.__srv_emergency_client.call_async(req) # async; 응답을 기다리는 동안 다른 작업 수행 가능 ! 
        self.get_logger().info(f'Sent emergency to TCP server via service: Robot {robot_id} - {emergency_type}')

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

