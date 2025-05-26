from nuri_msgs.msg import NuriBotTask
from nuri_system.robot.robot_state import RobotState
from nuri_system.robot.task_manger import TaskManager

class RobotManager():
    def __init__(self):
        self.robots = {}

    def register(self, id):        
        if id not in self.robots:
            self.robots[id] = RobotState(id)
            print(f'{id}번 로봇 등록', flush=True)

    def update_robot(self, id, status='대기', battery=0):
        first = False
        if id not in self.robots:
            self.register(id)
            first = True

        self.robots[id].update(status, int(battery))

        return first

    def update_cmd_vel(self, id, msg):
        if id not in self.robots:
            return
        
        self.robots[id].update_cmd_vel(msg)

    def update_location(self, id, position, orientation):
        if id not in self.robots:
            return

        self.robots[id].update_location(position, orientation)

    def update_marker_location(self, id, position, orientation):
        if id in self.robots:
            if self.robots[id].online:
                self.robots[id].update_location(position, orientation)

    def get_robot(self, id):
        return self.robots.get(id)
    
    def get_robot_marker_id(self, marker_id):
        for robot in self.get_all_robots():
            if robot.marker_id == marker_id:
                return robot
    
    def get_all_robots(self):
        return sorted(
            [robot for robot in self.robots.values() if robot.online],
            key=lambda robot: robot.id,
        )
    
    def get_disconnected_robot(self, timeout_sec):
        return [
            id for id, robot in self.robots.items() if robot.is_timed_out(timeout_sec)
        ]