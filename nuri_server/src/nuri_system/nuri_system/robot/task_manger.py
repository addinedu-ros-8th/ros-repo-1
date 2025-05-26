import time
import threading
from rclpy.node import Node

from datetime import datetime
from collections import deque

from geometry_msgs.msg import PoseStamped

from std_msgs.msg import String
from nuri_msgs.msg import NuriBotTask
from nuri_system.database.datbase_connection import NuriDatabase
from nuri_system.handler.socket_handler import SocketHandler
from nuri_system.network.packet.client_packet import ClientPacket

STATUS_PRIORITY = {"대기":0, "충전":1, "이동":2, "작업":3, "비상":4}

class TaskManager():
    def __init__(self, robot_manager, path_planner):
        self.node = Node('task_manager_node')
        self.robot_manager = robot_manager
        self.path_planner = path_planner
        self.task_list = deque()
        self.robots = []
        self.task_pub = {}

    def initialize_robot(self):
        self.robots = self.robot_manager.get_all_robots()
        if len(self.robots) == 0:
            return
        
        for robot in self.robots:
            topic = f'/robot{robot.id}/task'
            self.task_pub[robot.id] = self.node.create_publisher(NuriBotTask, topic, 10)

    def get_best_robot(self):
        self.update_priority()

        if len(self.robots) == 0:
            print("작업 가능 로봇 없음.", flush=True)
            return

        status = self.robots[0].status
        battery = self.robots[0].battery
        id = -1
        if (status != '대기' or status != '충전') and battery >= 0:
            id = self.robots[0]

        return id

    def update_priority(self):
        """ 상태에 점수를 부여하여 상태 점수와 배터리 잔량을 가지고 정렬 """
        
        self.robots.sort(
            key=lambda robot: (STATUS_PRIORITY.get(robot.status), -robot.battery)
        )

    def call_robot(self, qr_id):
        conn = NuriDatabase()
        query = "SELECT x_coord, y_coord FROM location_qrcode WHERE id = %s"
        result = conn.fetch_one(query, (qr_id+1,))

        # self.send_goal_pose(0.7367272973060608, 0.10234162956476212)

        # time.sleep(3)

        # self.send_goal_pose(result[0], result[1])

    def send_goal_pose(self, goal_x=0.0, goal_y=0.0):
        robot = self.get_best_robot()

        # goal_pose = PoseStamped()
        # goal_pose.header.frame_id = 'map'
        # goal_pose.pose.position.x = goal_x
        # goal_pose.pose.position.y = goal_y
        msg = NuriBotTask()
        msg.type = "작업"
        msg.x = goal_x
        msg.y = goal_y

        self.task_pub[robot.id].publish(msg)

        start = (robot.position.x, robot.position.y)
        goal = (goal_x, goal_y)

        # print('task ', self.path_planner.a_star(start, goal), flush=True)

        # thread = threading.Thread(target=self.check_goal_pose, args=(robot_id, goal_x, goal_y), daemon=True)
        # thread.start()

    def send_schedule(self, type, name=None, x=-1, y=-1):
        self.update_priority()

        robot = self.get_best_robot()

        msg = NuriBotTask()
        msg.type = type
        if name is not None:
            msg.name = name
        if x != -1:
            msg.x = x
        if y != -1:
            msg.y = y

        self.task_pub[robot.id].publish(msg)
        print(msg, flush=True)

        self.last_time = datetime.now()
        if type == "순찰":
            client_manager = SocketHandler.client_manager
            client_manager.broadcast(ClientPacket.patrol_mode(), 'device')

    def check_goal_pose(self, robot_id, goal_x, goal_y):
        robot = self.robot_manager.get_robot(robot_id)
        while True:
            dx = abs(goal_x - robot.position['x'])
            dy = abs(goal_y - robot.position['y'])

            if dx <= 0.2 and dy <= 0.2:
                self.node.get_logger().info('도착')
                break

            time.sleep(0.5)