import time
from datetime import datetime
from rclpy.node import Node

from nuri_msgs.msg import NuriBotState
from nuri_msgs.msg import NuriBotLocation
from std_msgs.msg import String
from std_msgs.msg import Bool

from nuri_system.database.datbase_connection import NuriDatabase
from nuri_system.handler.socket_handler import SocketHandler
from nuri_system.network.packet.client_packet import ClientPacket

class RobotHandler(Node):
    def __init__(self, robots, robot_manager):
        super().__init__('robot_handler_node')
        self.robot_manager = robot_manager
        self.state_sub = {}
        self.location_sub = {}

        self.update_state(robots)
        self.update_location(robots)
        self.update_patrol_time()
        self.last_time = None

        self.create_timer(1, self.check_robot_timeout)
        self.create_timer(1, self.send_robot_location)
        self.create_timer(1, self.send_robot_patrol)
    
    def update_state(self, robots):
        for robot_id in robots:
            topic = f'/robot{robot_id}/state'

            def callback(robot_id):
                def callback_2(msg):
                    if self.robot_manager.get_robot(robot_id) is None:
                        self.timer = self.create_timer(0.01, self.fetch_robot_status)
                    self.robot_manager.update_robot(robot_id, msg.status, msg.battery)
                return callback_2

            self.state_sub[robot_id] = self.create_subscription(NuriBotState, topic, callback(robot_id), 10)

    def update_location(self, robots):
        for robot_id in robots:
            topic = f'/robot{robot_id}/location'

            def callback(robot_id):
                def callback_2(msg):
                    self.robot_manager.update_location(robot_id, msg.x, msg.y)
                    # self.get_logger().info(f'{msg.x}, {msg.y}')
                return callback_2
            
            self.location_sub[robot_id] = self.create_subscription(NuriBotLocation, topic, callback(robot_id), 10)

    def check_robot_timeout(self):
        lost = self.robot_manager.get_disconnected_robot(5)
        for rid in lost:
            if self.robot_manager.get_robot(rid).online:
                self.robot_manager.get_robot(rid).online = False

    def send_robot_location(self):
        client_manager = SocketHandler.client_manager

        robots = self.robot_manager.get_all_robots()
        
        client_manager.broadcast(ClientPacket.send_robot_location(robots))

    def fetch_robot_status(self):
        conn = NuriDatabase.get_instance()

        result = None
        try:
            query = "SELECT type FROM robot_status"
            result = conn.fetch_all(query)
        except Exception as e:
            self.get_logger().info(f"[SQL ERROR] {e}")

        if result is not None:
            status_list = ""
            for row in result:
                status_list += row[0] + ","
            msg_str = String()
            msg_str.data = status_list[:-1]
            pub = self.create_publisher(String, 'status_list', 10)
            time.sleep(0.1)
            pub.publish(msg_str)

        self.timer.cancel()

    def update_patrol_time(self):
        conn = NuriDatabase.get_instance()

        query = "SELECT scheduled_time FROM schedule order by scheduled_time ASC"
        result = conn.fetch_all(query)

        self.patrol_time = {}

        for row in result:
            self.patrol_time[str(row[0])[:-3]] = False

    def send_robot_patrol(self):
        now = datetime.now()
        # now = datetime(2025, 5, 13, 0, 0)
        
        for time, complete in self.patrol_time.items():
            # time = datetime.strptime(time, "%H:%M")
            if now == time:
                self.patrol_time[time] = True
                pub = self.create_publisher(Bool, 'patrol', 10)
                msg = Bool()
                msg.data = True
                pub.publish(msg)
                self.last_time = datetime.now()
                break

        if self.last_time is not None:
            if (now.date() - self.last_time.date()).days >= 1:
                self.update_patrol_time()