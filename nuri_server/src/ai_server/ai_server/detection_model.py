import cv2 
import torch 
from ultralytics import YOLO 
from collections import deque
import logging

class ObjectDetector: 
    """
    객체 감지 by YOLOv11 from Ultralytics 
    """
    def __init__(self, model_size='small', conf_thres=0.25, 
                 iou_thres=0.45, classes=None, device=None, ros_logger=None):
        """
        Args:
            model_size (str): Model size ('nano', 'small', 'medium', 'large', 'extra')
            conf_thres (float): Confidence threshold for detections
            iou_thres (float): IoU threshold for NMS
            classes (list): List of classes to detect (None for all classes)
            device (str): Device to run inference on ('cuda', 'cpu', 'mps')
        """
        
        self.logger = ros_logger if ros_logger is not None else logging.getLogger(__name__)
        if ros_logger is None:
            logging.basicConfig(level=logging.INFO) # 기본 로거 레벨 설정 (ros_logger가 없을 때)
        
        # Determine device 
        if device is None:
            if torch.cuda.is_available(): device = 'cuda'
            else: device = 'cpu'

        self.device = device
        self.logger.info(f"Using device: {self.device} for object detection")

        # Map model size to model name 
        model_map = {
            'nano': 'yolo11n',
            'small': 'yolo11s',
            'medium': 'yolo11m',
            'large': 'yolo11l',
            'extra': 'yolo11x'
        }

        model_name = model_map.get(model_size.lower(), model_map['small'])

        # Load model 
        try:
            self.model = YOLO(model_name)
            self.logger.info(f"Loaded YOLOv11 {model_size} model on {self.device}") 
        except Exception as e:
            self.logger.error(f"Error loading model: {e}")
            self.logger.info("Trying to load with default settings...")
            self.model = YOLO(model_name)
        
        # Set model params 
        self.model.overrides['conf'] = conf_thres
        self.model.overrides['iou'] = iou_thres
        self.model.overrides['agnostic_nms'] = False
        self.model.overrides['max_det'] = 1000

        if classes is not None:
            self.model.overrides['classes'] = classes 
        
        # init tracking trajectories
        self.tracking_trajectories = {}

    def detect_only(self, img, track=True):
        """
        그리기 없이 검출만 하는 순수한 함수 (성능 최적화)
        Args:
            image (numpy.ndarray): Input image (BGR format)
            track (bool): Whether to track objects across frames
            
        Returns:
            list: List of detections [bbox, score, class_id, object_id]
        """
        detections = []

        try:
            if track:  
                results = self.model.track(img, verbose=False, device=self.device, persist=True)
            else:      
                results = self.model.predict(img, verbose=False, device=self.device) 

        except RuntimeError as e:
            raise 
        
        if track:
            # 검출 결과만 수집 (그리기 없음!)
            for predictions in results:
                if predictions is None:
                    continue
                
                if predictions.boxes is None:
                    continue
                
                for bbox in predictions.boxes:
                    scores = bbox.conf
                    classes = bbox.cls
                    bbox_coords = bbox.xyxy
                    
                    if hasattr(bbox, 'id') and bbox.id is not None:
                        ids = bbox.id
                    else:
                        ids = [None] * len(scores)
                    
                    # 검출 정보만 저장 (그리기 X)
                    for score, class_id, bbox_coord, id_ in zip(scores, classes, bbox_coords, ids):
                        xmin, ymin, xmax, ymax = bbox_coord.cpu().numpy()
                        
                        detections.append([
                            [xmin, ymin, xmax, ymax],  # bbox
                            float(score),              # confidence score
                            int(class_id),             # class id
                            int(id_) if id_ is not None else None  # object id
                        ])
                        
                        # trajectory는 ID=1만 저장 (성능 최적화)
                        if id_ is not None and int(id_) == 1:
                            centroid_x = (xmin + xmax) / 2
                            centroid_y = (ymin + ymax) / 2
                            
                            if 1 not in self.tracking_trajectories:
                                self.tracking_trajectories[1] = deque(maxlen=10)
                            
                            self.tracking_trajectories[1].append((centroid_x, centroid_y))
        else:
            # Non-tracking mode
            for predictions in results:
                if predictions is None:
                    continue
                
                if predictions.boxes is None:
                    continue
                
                for bbox in predictions.boxes:
                    scores = bbox.conf
                    classes = bbox.cls
                    bbox_coords = bbox.xyxy
                    
                    for score, class_id, bbox_coord in zip(scores, classes, bbox_coords):
                        xmin, ymin, xmax, ymax = bbox_coord.cpu().numpy()
                        
                        detections.append([
                            [xmin, ymin, xmax, ymax],
                            float(score),
                            int(class_id),
                            None
                        ])
        
        return detections
    # def detect(self, img, track=True):
    #     """
    #     Detect objs in an image 
    #     Args:
    #         image (numpy.ndarray): Input image (BGR format)
    #         track (bool): Whether to track objects across frames
            
    #     Returns:
    #         tuple: (annotated_image, detections)
    #             - annotated_image (numpy.ndarray): Image with detections drawn
    #             - detections (list): List of detections [bbox, score, class_id, object_id]
    #     """
    #     detections = []

    #     # make a copy of the image for annotation 
    #     annotated_image = img.copy()

    #     try:
    #         if track:  results = self.model.track(img, verbose= False, device = self.device, persist=True)
    #         else:      results = self.model.predict(img, verbose= False, device = self.device) 

    #     except RuntimeError as e:
    #         raise 
        
    #     if track:
    #         # Clean up trajectories for objects that are no longer tracked
    #         for id_ in list(self.tracking_trajectories.keys()):
    #             if id_ not in [int(bbox.id) for predictions in results if predictions is not None 
    #                           for bbox in predictions.boxes if bbox.id is not None]:
    #                 del self.tracking_trajectories[id_]
            
    #         # Process results
    #         for predictions in results:
    #             if predictions is None:
    #                 continue
                
    #             if predictions.boxes is None:
    #                 continue
                
    #             # Process boxes
    #             for bbox in predictions.boxes:
    #                 # Extract information
    #                 scores = bbox.conf
    #                 classes = bbox.cls
    #                 bbox_coords = bbox.xyxy
                    
    #                 # Check if tracking IDs are available
    #                 if hasattr(bbox, 'id') and bbox.id is not None:
    #                     ids = bbox.id
    #                 else:
    #                     ids = [None] * len(scores)
                    
    #                 # Process each detection
    #                 for score, class_id, bbox_coord, id_ in zip(scores, classes, bbox_coords, ids):
    #                     xmin, ymin, xmax, ymax = bbox_coord.cpu().numpy()
                        
    #                     # Add to detections list
    #                     detections.append([
    #                         [xmin, ymin, xmax, ymax],  # bbox
    #                         float(score),              # confidence score
    #                         int(class_id),             # class id
    #                         int(id_) if id_ is not None else None  # object id
    #                     ])
                        
    #                     # Draw bounding box
    #                     cv2.rectangle(annotated_image, 
    #                                  (int(xmin), int(ymin)), 
    #                                  (int(xmax), int(ymax)), 
    #                                  (0, 0, 225), 2)
                        

    #                     # 텍스트 준비
    #                     label = f"ID: {int(id_) if id_ is not None else 'N/A'} {predictions.names[int(class_id)]} {float(score):.2f}"

    #                     # 더 큰 폰트 크기 설정
    #                     font_scale = 0.8  # 원래 0.5에서 0.8로 증가
    #                     font_thickness = 2  # 원래 1에서 2로 증가

    #                     # 텍스트 크기 계산
    #                     text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
    #                     text_width, text_height = text_size[0]

    #                     # 바운딩 박스 중앙 위치 계산
    #                     center_x = int(xmin + (xmax - xmin) / 2)
    #                     center_y = int(ymin + (ymax - ymin) / 2)

    #                     # 텍스트를 위한 배경 위치 계산
    #                     text_x = center_x - text_width // 2
    #                     text_y = center_y + text_height // 2

    #                     # 텍스트 배경 그리기 (약간 더 큰 패딩 추가)
    #                     padding = 10
    #                     cv2.rectangle(annotated_image,
    #                                 (text_x - padding, text_y - text_height - padding),
    #                                 (text_x + text_width + padding, text_y + padding),
    #                                 (30, 30, 30), cv2.FILLED)

    #                     # 텍스트 그리기
    #                     cv2.putText(annotated_image, 
    #                                 label,
    #                                 (text_x, text_y),
    #                                 cv2.FONT_HERSHEY_SIMPLEX, 
    #                                 font_scale, 
    #                                 (255, 255, 255), 
    #                                 font_thickness)
                        
    #                     # Update tracking trajectories
    #                     if id_ is not None:
    #                         centroid_x = (xmin + xmax) / 2
    #                         centroid_y = (ymin + ymax) / 2
                            
    #                         if int(id_) not in self.tracking_trajectories:
    #                             self.tracking_trajectories[int(id_)] = deque(maxlen=10)
                            
    #                         self.tracking_trajectories[int(id_)].append((centroid_x, centroid_y))
            
    #         # Draw trajectories
    #         for id_, trajectory in self.tracking_trajectories.items():
    #             for i in range(1, len(trajectory)):
    #                 thickness = int(2 * (i / len(trajectory)) + 1)
    #                 cv2.line(annotated_image, 
    #                         (int(trajectory[i-1][0]), int(trajectory[i-1][1])), 
    #                         (int(trajectory[i][0]), int(trajectory[i][1])), 
    #                         (255, 255, 255), thickness)
        
    #     else:
    #         # Process results for non-tracking mode
    #         for predictions in results:
    #             if predictions is None:
    #                 continue
                
    #             if predictions.boxes is None:
    #                 continue
                
    #             # Process boxes
    #             for bbox in predictions.boxes:
    #                 # Extract information
    #                 scores = bbox.conf
    #                 classes = bbox.cls
    #                 bbox_coords = bbox.xyxy
                    
    #                 # Process each detection
    #                 for score, class_id, bbox_coord in zip(scores, classes, bbox_coords):
    #                     xmin, ymin, xmax, ymax = bbox_coord.cpu().numpy()
                        
    #                     # Add to detections list
    #                     detections.append([
    #                         [xmin, ymin, xmax, ymax],  # bbox
    #                         float(score),              # confidence score
    #                         int(class_id),             # class id
    #                         None                       # object id (None for no tracking)
    #                     ])
                        
    #                     # Draw bounding box
    #                     cv2.rectangle(annotated_image, 
    #                                  (int(xmin), int(ymin)), 
    #                                  (int(xmax), int(ymax)), 
    #                                  (0, 0, 225), 2)
                        
    #                     # Add label
    #                     label = f"{predictions.names[int(class_id)]} {float(score):.2f}"
    #                     text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    #                     dim, baseline = text_size[0], text_size[1]
    #                     cv2.rectangle(annotated_image, 
    #                                  (int(xmin), int(ymin)), 
    #                                  (int(xmin) + dim[0], int(ymin) - dim[1] - baseline), 
    #                                  (30, 30, 30), cv2.FILLED)
    #                     cv2.putText(annotated_image, label, 
    #                                (int(xmin), int(ymin) - 7), 
    #                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
    #     return annotated_image, detections
    
    def get_class_names(self):
        """
        Get the names of the classes that the model can detect
        
        Returns:
            list: List of class names
        """
        return self.model.names 