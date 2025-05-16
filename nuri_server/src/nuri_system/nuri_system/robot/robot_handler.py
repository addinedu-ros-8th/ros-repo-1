import time
import numpy as np
import cv2
from datetime import datetime
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nuri_msgs.msg import NuriBotState
from nuri_msgs.msg import NuriBotLocation
from nuri_msgs.msg import NuriBotSchedule

from nuri_system.database.datbase_connection import NuriDatabase
from nuri_system.handler.socket_handler import SocketHandler
from nuri_system.network.packet.client_packet import ClientPacket

class RobotHandler(Node):
    def __init__(self, robots, robot_manager):
        super().__init__('robot_handler_node')
        self.robot_manager = robot_manager

        """ 로봇의 Topic을 구독하기 위한 함수들 """
        self.state_sub = {}
        self.location_sub = {}
        self.request_status_sub = {}
        self.map_sub = {}
        self.update_state(robots)
        self.update_location(robots)
        self.send_status_list(robots)
        self.update_map(robots)

        self.last_time = None
        self.update_schedule_time()
        self.fetch_robot_status()

        self.create_timer(1, self.check_robot_timeout)
        self.create_timer(1, self.send_robot_patrol)
        self.map_info = {}

    def update_map(self, robots):
        for robot_id in robots:
            topic = f'/robot{robot_id}/map'

            def callback():
                def callback_2(msg):
                    self.update_map_data(msg)
                return callback_2

            self.map_sub[robot_id] = self.create_subscription(OccupancyGrid, topic, callback(), 10)

    def update_map_data(self, msg):
        new_info = {
            'resolution': msg.info.resolution,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
            'width': float(msg.info.width),
            'height': float(msg.info.height)
        }

        if self.map_info != new_info:
            self.map_info = new_info
            self.occupancy_to_image(msg)
            img = self.occupancy_to_image(msg)
            _, self.map_jpg = cv2.imencode('.jpg', img)
    
    def update_state(self, robots):
        for robot_id in robots:
            topic = f'/robot{robot_id}/state'

            def callback(robot_id):
                def callback_2(msg):
                    self.robot_manager.update_robot(robot_id, msg.status, msg.battery)
                return callback_2

            self.state_sub[robot_id] = self.create_subscription(NuriBotState, topic, callback(robot_id), 10)

    def update_location(self, robots):
        for robot_id in robots:
            topic = f'/robot{robot_id}/tracked_pose'

            def callback(robot_id):
                def callback_2(msg):
                    self.robot_manager.update_location(robot_id, msg.pose.position, msg.pose.orientation)
                    if len(self.map_info) != 0:
                        SocketHandler.client_manager.broadcast(ClientPacket.send_robot_location(self.robot_manager.get_all_robots()))
                return callback_2
            
            self.location_sub[robot_id] = self.create_subscription(PoseStamped, topic, callback(robot_id), 10)

    def check_robot_timeout(self):
        lost = self.robot_manager.get_disconnected_robot(5)
        for rid in lost:
            if self.robot_manager.get_robot(rid).online:
                self.robot_manager.get_robot(rid).online = False

    def fetch_robot_status(self):
        conn = NuriDatabase.get_instance()

        result = None
        try:
            query = "SELECT type FROM robot_status"
            result = conn.fetch_all(query)
        except Exception as e:
            self.get_logger().info(f"[SQL ERROR] {e}")

        self.status_list = [row[0] for row in result]

    def update_schedule_time(self):
        conn = NuriDatabase.get_instance()

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

    def send_robot_patrol(self):
        now = datetime.now().strftime('%H:%M')
        msg = NuriBotSchedule()
        robot_id = -1
        
        for id, value in self.schedule_time.items():
            type = "순찰" if id == 4 else "산책"
            msg.type = type
            
            if now == value['time'] and not value['success']:
                value['success'] = True
                if type == "산책":
                    conn = NuriDatabase.get_instance()
                    query = "SELCT b.x_coord, b.y_coord FROM residents r, bed b WHERE r.bed_id = b.id AND r.name = %s"
                    result = conn.fetch_one(query, (value['name'],))

                    msg.x = result[0]
                    msg.y = result[1]

                robot_id = self.robot_manager.get_idle_robot()

            pub = self.create_publisher(NuriBotSchedule, f'/robot{robot_id}/schedule', 10)
            pub.publish(msg)
            self.last_time = datetime.now()
            client_manager = SocketHandler.client_manager
            client_manager.broadcast(ClientPacket.patrol_mode(), 'device')
            break

        if self.last_time is not None:
            if (now.date() - self.last_time.date()).days >= 1:
                self.update_schedule_time()

    def send_status_list(self, robots):
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

    def occupancy_to_image(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        img = np.zeros((height, width), dtype=np.uint8)
        img[data == -1] = 128  # unknown
        img[data == 0] = 255   # free
        img[data == 100] = 0   # occupied

        img = np.fliplr(img)

        # cv2.imwrite('map.jpg', img)
        return img