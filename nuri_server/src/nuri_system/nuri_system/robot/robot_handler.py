import time, os
import numpy as np
import cv2
import math
from datetime import datetime
from rclpy.time import Duration
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped

from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from nuri_msgs.msg import NuriBotState
from nuri_msgs.msg import NuriBotTask
from nuri_msgs.srv import NuriBotCall

from nuri_system.database.datbase_connection import NuriDatabase
from nuri_system.robot.task_manger import TaskManager
from nuri_system.robot.path_planner import PathPlanner
from nuri_system.robot.robot_manager import RobotManager
from nuri_system.handler.socket_handler import SocketHandler
from nuri_system.network.packet.client_packet import ClientPacket

class RobotHandler(Node):
    def __init__(self, robots):
        super().__init__('robot_handler_node')
        self.robot_manager = RobotManager()
        self.path_planner = PathPlanner(self.robot_manager)

        self.call_service = self.create_service(NuriBotCall, 'call', self.call_nuri)

        """ 로봇의 Topic을 구독하기 위한 함수들 """
        self.state_sub = {}
        self.location_sub = {}
        self.request_status_sub = {}
        self.map_sub = {}
        self.update_state(robots)
        self.update_location(robots)
        self.send_status_list(robots)

        # 맵 업데이트가 필요한 경우 주석 제거 self.update_map(robots)
        # self.update_map(robots)

        self.last_time = None
        self.update_schedule_time()
        self.fetch_robot_status()

        self.create_timer(1, self.check_robot_timeout)
        self.create_timer(1, self.assign_schedule)

        self.task_manager = TaskManager(self.robot_manager, self.path_planner)
        self.server_ready_sub = self.create_subscription(Bool, '/server_ready', self.server_ready_callback, 10)

        self.aruco_sub = self.create_subscription(String, '/aruco_pose', self.aruco_callback, 10)

    def aruco_callback(self, msg):
        arcuo_id, x, y = str(msg.data).split(',')

        robot = self.robot_manager.get_robot_marker_id(int(arcuo_id))
        if robot is not None:
            robot.update_marker_location(float(x), float(y))

    def server_ready_callback(self, msg):
        if msg.data:
            self.robot_manager.server_ready = True

    def update_map(self, robots):
        for robot_id in robots:
            topic = f'/robot{robot_id}/map'

            def callback():
                def callback_2(msg):
                    self.update_map_data(msg)
                return callback_2

            self.map_sub[robot_id] = self.create_subscription(OccupancyGrid, topic, callback(), 10)

    def update_map_data(self, msg):
        """ 로봇의 /map을 받아와 dict를 구성 후 기존 dict와 비교하여 업데이트 """

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin  # geometry_msgs/Pose
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        np.savez('map_info.npz',
            grid=grid,
            width=width,
            height=height,
            resolution=resolution,
            origin_position=np.array([
                origin.position.x,
                origin.position.y,
                origin.position.z
            ]),
            origin_orientation=np.array([
                origin.orientation.x,
                origin.orientation.y,
                origin.orientation.z,
                origin.orientation.w
            ])
        )
        self.get_logger().info('맵 저장 완료')
    
    def update_state(self, robots):
        """ 각 로봇의 /state를 받아와 연결된 클라이언트에 상태 전송 """
        for robot_id in robots:
            topic = f'/robot{robot_id}/state'

            def callback(robot_id):
                def callback_2(msg):
                    regist = self.robot_manager.update_robot(robot_id, msg.status, msg.battery)
                    if regist:
                        self.task_manager.initialize_robot()
                        print("로봇 초기화", flush=True)
                    SocketHandler.client_manager.broadcast(ClientPacket.send_robot_list(self.robot_manager.get_all_robots()))
                return callback_2

            self.state_sub[robot_id] = self.create_subscription(NuriBotState, topic, callback(robot_id), 10)

    def update_location(self, robots):
        """ 각 로봇의 /tracked_pose를 받아와 연결된 클라이언트에 위치 전송 """
        for robot_id in robots:
            topic = f'/robot{robot_id}/tracked_pose'

            def callback(robot_id):
                def callback_2(msg):
                    self.robot_manager.update_location(robot_id, msg.pose.position, msg.pose.orientation)
                    SocketHandler.client_manager.broadcast(ClientPacket.send_robot_location(self.robot_manager.get_all_robots()))
                return callback_2
            
            self.location_sub[robot_id] = self.create_subscription(PoseStamped, topic, callback(robot_id), 10)

    def check_robot_timeout(self):
        """ 마지막 상태 업데이트 시간으로부터 5초간 업데이트 되지 않으면 오프라인 처리 """
        lost = self.robot_manager.get_disconnected_robot(5)
        for rid in lost:
            if self.robot_manager.get_robot(rid).online:
                self.robot_manager.get_robot(rid).online = False

    def fetch_robot_status(self):
        """ DB에서 로봇 상태 목록을 꺼내옴 """
        conn = NuriDatabase()

        result = None
        try:
            query = "SELECT type FROM robot_status"
            result = conn.fetch_all(query)
        except Exception as e:
            self.get_logger().info(f"[SQL ERROR] {e}")

        self.status_list = [row[0] for row in result]

    def update_schedule_time(self):
        """ 
        DB에 등록된 스케줄을 가져와 업데이트 
        2 : 산책, 4: 순찰
        """
        conn = NuriDatabase()

        query = "SELECT DATE_FORMAT(scheduled_time, '%H:%i') FROM schedule WHERE job_id = 4 order by scheduled_time ASC"
        result = conn.fetch_all(query)

        self.schedule_time = {}

        for row in result:
            self.schedule_time[4] = {'time':row[0], 'success':False}

        query = """
        SELECT DATE_FORMAT(s.scheduled_time, '%H:%i'), r.name
        FROM schedule s, walk_schedule w, residents r
        WHERE s.id = w.schedule_id
        AND w.user_id = r.id
        """
        result = conn.fetch_all(query)

        for row in result:
            self.schedule_time[2] = {'name':row[1], 'time':row[0], 'success':False}

    def assign_schedule(self):
        """ 스케줄에 등록된 작업, 시간에 따라 로봇에게 작업 할당 """
        now = datetime.now().strftime('%H:%M')
        x, y, name = -1, -1, ""
        
        for id, value in self.schedule_time.items():
            type = "순찰" if id == 4 else "산책"
            
            if now == value['time'] and not value['success']:
                value['success'] = True
                if type == "산책":
                    name = value['name']
                    conn = NuriDatabase()
                    query = "SELCT b.x_coord, b.y_coord FROM residents r, bed b WHERE r.bed_id = b.id AND r.name = %s"
                    result = conn.fetch_one(query, (name,))

                    # self.send_command(type)
                    x = result[0]
                    y = result[1]
                self.task_manager.send_schedule(type, name, x, y)

        if self.last_time is not None:
            if (now.date() - self.last_time.date()).days >= 1:
                self.update_schedule_time()

    def send_status_list(self, robots):
        """ 요청이 들어온 로봇에게 상태 목록 전송"""

        for robot_id in robots:
            topic = f'/robot{robot_id}/request_status'

            def callback(robot_id):
                def callback_2(msg):
                     if msg.data:
                        pub = self.create_publisher(String, f'/robot{robot_id}/response_status', 10)
                        response_msg = String()
                        for row in self.status_list:
                            response_msg.data += row + ","
                        
                        pub.publish(response_msg[:-1])
                return callback_2
                    
        self.request_status_sub[robot_id] = self.create_subscription(Bool, topic, callback(robot_id), 10)
    
    def call_nuri(self, request, response):
        qr_id = request.id

        try:
            response.success = True

            self.task_manager.call_robot(qr_id)
        except Exception as e:
            response.success = False
            print(e, flush=True)

        return response

    def send_command(self, command):
        msg = String()
        msg.data = command

        pub = self.create_publisher(String, '/task', 10)
        pub.publish(msg)