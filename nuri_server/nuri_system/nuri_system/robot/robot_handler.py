import rclpy
from rclpy.node import Node

from nuri_msgs.msg import RobotState

class RobotHandler(Node):
    def __init__(self, robots, robot_manager):
        super().__init__('robot_handler_node')
        self.robot_manager = robot_manager
        self.state_sub = {}

        self.update_state(robots)

        self.create_timer(1, self.check_robot_timeout)
    
    def update_state(self, robots):
        for robot_id in robots:
            topic = f"/robot{robot_id}/state"

            def callback(robot_id):
                def callback_2(msg):
                    # self.get_logger().info(f"{robot_id}, {msg.status}, {msg.battery}")
                    self.robot_manager.update_robot(robot_id, msg.status, msg.battery)
                return callback_2

            self.state_sub[robot_id] = self.create_subscription(RobotState, topic, callback(robot_id), 10)

    def check_robot_timeout(self):
        lost = self.robot_manager.get_disconnected_robot(5)
        for rid in lost:
            if self.robot_manager.get_robot(rid).online:
                self.robot_manager.get_robot(rid).online = False