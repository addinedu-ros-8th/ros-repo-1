import cv2
import os
import math
import time
import numpy as np
import threading
import tf2_ros
import rclpy
from rclpy.node import Node
from collections import deque

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from tf_transformations import euler_from_quaternion

class InitialPose(Node):
    def __init__(self, robots, robot_manager):
        super().__init__("initial_pose_node")

        self.robot_manager = robot_manager

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # 카메라 행렬 데이터
        self.mtx = np.load("calibration_matrix.npy")
        self.dist = np.load("distortion_coefficients.npy")

        # 카메라 좌표와 SLAM 좌표 맵핑
        self.pixel_pts = np.array([[645, 43], [40, 58], [45, 421], [648, 418], [451, 285], [223, 34]], dtype=np.float32)
        self.slam_pts = np.array([[-0.09, -0.316], [2.365, -0.324], [2.359, 1.121], [-0.11, 1.211],[0.735, 0.732], [1.1414, -0.3196]], dtype=np.float32)
        self.H, _ = cv2.findHomography(self.pixel_pts, self.slam_pts, method=cv2.RANSAC)

        # ArUco 마커 준비
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_100)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

        # 글로벌 카메라 시작
        threading.Thread(target=self.read_frame, daemon=True).start()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.init_sub = {}
        self.cmd_vel_sub = {}
        self.get_cmd_vel(robots)
        self.create_timer(1, self.initial_pose)

        self.pose_pub = {}

        self.drift_count = {}
        self.drift_history = {}

    def initial_pose(self):
        for robot in self.robot_manager.get_all_robots():
            if robot.id not in self.pose_pub:
                self.pose_pub[robot.id] = self.create_publisher(PoseWithCovarianceStamped, f'/robot{robot.id}/initial_pose', 10)

        for robot in self.robot_manager.get_all_robots():
                if not robot.marker_position:
                    return
                
                cmd_vel = robot.cmd_vel
                if cmd_vel is None or (cmd_vel.linear.x == 0.0 and cmd_vel.angular.z == 0.0):
                    msg = PoseWithCovarianceStamped()
                    # msg.header.stamp.sec = 0
                    # msg.header.stamp.nanosec = 0
                    msg.header.frame_id = 'map'


                    diff_x = robot.marker_position.x - robot.tracked_position.x
                    diff_y = robot.marker_position.y - robot.tracked_position.y
                    pose_yaw = self.get_yaw_from_quaternion(robot.tracked_orientation)
                    marker_yaw = self.get_yaw_from_quaternion(robot.marker_orientation)
                    diff_yaw = marker_yaw - pose_yaw


                    # drift 판단 로직
                    is_drift = not (-0.97 <= diff_x <= -0.055 and -0.02 <= diff_y <= 0.055 and -0.05 <= diff_yaw <= 0.05)

                    # 큐 생성 및 기록
                    if robot.id not in self.drift_history:
                        self.drift_history[robot.id] = deque(maxlen=10)

                    self.drift_history[robot.id].append(is_drift)

                    # drift 상태 판단
                    history = self.drift_history[robot.id]

                    if len(history) == 10:
                        drift_count = sum(history)
                        if drift_count >= 5:
                            print(f"⚠️ DRIFT DETECTED for robot {robot.id} ({drift_count}/10)", flush=True)

                            msg.pose.pose.position.x = robot.marker_position.x - 0.02
                            msg.pose.pose.position.y = robot.marker_position.y
                            msg.pose.pose.orientation.x = robot.marker_orientation.x
                            msg.pose.pose.orientation.y = robot.marker_orientation.y
                            msg.pose.pose.orientation.z = robot.marker_orientation.z - 0.04
                            msg.pose.pose.orientation.w = robot.marker_orientation.w

                            msg.pose.covariance = [0.0] * 36
                            msg.pose.covariance[0] = 0.25
                            msg.pose.covariance[6] = 0.25
                            msg.pose.covariance[35] = 0.0685

                            self.pose_pub[robot.id].publish(msg)
                            # self.drift_history[robot.id].clear()  # 초기화

                        else:
                            print(f"✅ drift check reset for robot {robot.id} ({drift_count}/10)", flush=True)

                        self.drift_history[robot.id].clear()


    def get_yaw_from_quaternion(self, orientation):
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(q)
        return yaw  # 단위: 라디안

    def get_cmd_vel(self, robots):
        for robot in robots:
            topic = f'/robot{robot}/cmd_vel'

            def callback(robot_id):
                def callback_2(msg):
                    self.robot_manager.update_cmd_vel(robot_id, msg)
                return callback_2
            
            self.cmd_vel_sub[robot] = self.create_subscription(Twist, topic, callback(robot), 10)
            
    def pixel_to_slam(self, x, y):
        src = np.array([[[x, y]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(src, self.H)
        return dst[0][0]

    def read_frame(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            # 마커 인식을 위한 Frame 밝기 낮추기
            frame = cv2.subtract(frame, np.full(frame.shape, 50, dtype=np.uint8))

            cropped_img = frame[110: 110 + 450, 237: 237 + 680]
            corners, ids, _ = self.detector.detectMarkers(cropped_img)
            if ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, self.mtx, self.dist)
                for i, marker in enumerate(corners):
                    c = marker[0]

                    cx = int(np.mean(c[:, 0]))
                    cy = int(np.mean(c[:, 1]))
                    # slam_y, slam_x = pixel_to_slam(cx, cy)
                    slam_x, slam_y = self.pixel_to_slam(cx, cy)

                    rotation_matrix, _ = cv2.Rodrigues(rvecs[i][0])
                    marker_yaw_camera = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

                    # 카메라 yaw 보정 (예: 0이면 생략, 90도면 np.pi/2)
                    camera_yaw = np.deg2rad(90)  # 필요에 따라 수정
                    slam_yaw = camera_yaw + marker_yaw_camera

                    qx, qy, qz, qw = quaternion_from_euler(0, 0, slam_yaw)

                    # text = f'{ids} {slam_x:.3f}, {slam_y:.3f}'
                    # cv2.putText(cropped_img, text, (cx-50, cy), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1)


                    # self.get_logger().info(
                    #     f'x: {slam_x:.3f}, y: {slam_y:.3f} / q: x={qx:.3f}, y={qy:.3f}, z={qz:.3f}, w={qw:.3f}'
                    # )
                    robot = self.robot_manager.get_robot_marker_id(ids[0])
                    if robot is None:
                        continue

                    position = PoseStamped()
                    position.pose.position.x = float(slam_x)
                    position.pose.position.y = float(slam_y)
                    position.pose.orientation.x = float(qx)
                    position.pose.orientation.y = float(qy)
                    position.pose.orientation.z = float(qz)
                    position.pose.orientation.w = float(qw)

                    robot.update_marker_location(position)

                    # test = f'x:{(slam_x - self.pose_x):.3f} y:{slam_y - self.pose_y:.3f} qx:{qx - self.orient_x:.3f} qy:{qy - self.orient_y:.3f} qz:{qz - self.orient_z:.3f} qw:{qw - self.orient_w:.3f}'
                    # test = f'{slam_x} {self.pose_x}'
                    # self.get_logger().info(f'{test}')
            # cv2.imshow('Frmae', cropped_img)
            # cv2.waitKey(1)