import socket
import struct
import cv2, os
import numpy as np
import math
import rclpy
import threading
from queue import PriorityQueue
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler
from ament_index_python.packages import get_package_share_directory

mtx = np.load("calibration_matrix.npy")
dist = np.load("distortion_coefficients.npy")

# 좌표 매핑 (웹캠 픽셀 ↔ SLAM 좌표)
pixel_pts = np.array([[645, 43], [40, 58], [45, 421], [648, 418], [451, 285], [223, 34]], dtype=np.float32)
slam_pts = np.array([[-0.09, -0.316], [2.365, -0.324], [2.359, 1.121], [-0.11, 1.211],[0.735, 0.732], [1.1414, -0.3196]], dtype=np.float32)
H, _ = cv2.findHomography(pixel_pts, slam_pts, method=cv2.RANSAC)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_100)
detector_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

def pixel_to_slam(x, y):
    src = np.array([[[x, y]]], dtype=np.float32)
    dst = cv2.perspectiveTransform(src, H)
    return dst[0][0]

class ArucoTCPNode(Node):
    def __init__(self):
        super().__init__('aruco_tcp_node')
        self.get_logger().info("[STARTED] ArucoTCPNode")

        self.cap = cv2.VideoCapture(1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.position = {}

        self.pub = self.create_publisher(String, '/aruco_pose', 10)

        threading.Thread(target=self.process_frame, daemon=True).start()

        self.pose_pub = {}

    def init_pose_publihser(self):
        for id in (1, 97):
            self.pose_pub[id] = self.create_publisher(PoseWithCovarianceStamped, f'/robot{id}/init_pose', 10)

    def click_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(x, y, flush=True)


    def process_frame(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                return
            
            cv2.namedWindow("Camera")
            cv2.setMouseCallback("Camera", self.click_callback, param={"node": self})

            frame = cv2.subtract(frame, np.full(frame.shape, 50, dtype=np.uint8))

            h, w = frame.shape[:2]
            new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
            map1, map2 = cv2.initUndistortRectifyMap(mtx, dist, None, new_camera_mtx, (w, h), 5)
            x, y, w_roi, h_roi = roi

            src_pts = np.array([[170, 100], [900, 100], [900, 550], [170, 550]], dtype=np.float32)
            output_size = (600, 500)
            dst_pts = np.array([[0, 0], [output_size[0]-1, 0], [output_size[0]-1, output_size[1]-1], [0, output_size[1]-1]], dtype=np.float32)
            M = cv2.getPerspectiveTransform(src_pts, dst_pts)

            # undistorted = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
            undistorted = cv2.undistort(frame, mtx, dist)
            undistorted_cropped = undistorted[y:y+h_roi, x:x+w_roi]
            warped = cv2.warpPerspective(undistorted_cropped, M, output_size)
            # cropped_img = frame[110: 110 + 450, 237: 237 + 680]
            
            corners, ids, _ = detector.detectMarkers(warped)
            if ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, mtx, dist)
                for i, marker in enumerate(corners):
                    marker_id = ids[i][0]
                    c = marker[0]
                    center = np.mean(c, axis=0).astype(int)
                    cx = int(np.mean(c[:, 0]))
                    cy = int(np.mean(c[:, 1]))
                    # slam_y, slam_x = pixel_to_slam(cx, cy)
                    slam_x, slam_y = pixel_to_slam(cx, cy)

                    rotation_matrix, _ = cv2.Rodrigues(rvecs[i][0])
                    marker_yaw_camera = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

                    dx = c[0][0] - c[3][0]
                    dy = c[0][1] - c[3][1]
                    angle_rad = np.arctan2(dy, dx)

                    tip_x = int(center[0] + 50 * np.cos(angle_rad))
                    tip_y = int(center[1] + 50 * np.sin(angle_rad))
                    cv2.arrowedLine(warped, center, (tip_x, tip_y), (255, 0, 0), 2)
                    # text = f'{cx}, {cy}, {camera_yaw}'
                    text = f'{ids[i][0]}'
                    cv2.putText(warped, text, (cx-50, cy), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1)


                    self.position[ids[i][0]] = {
                        "x" : slam_x,
                        "y" : slam_y,
                        "yaw" : marker_yaw_camera
                    }
                    msg = String()
                    msg.data = f'{ids[i][0]},{slam_x},{slam_y}'
                    self.pub.publish(msg)


            cv2.imshow('Camera', warped)
            cv2.waitKey(1)

def main():
    rclpy.init()
    node = ArucoTCPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
