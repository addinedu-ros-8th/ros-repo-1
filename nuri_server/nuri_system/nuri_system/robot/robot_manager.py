from nuri_system.robot.robot_state import RobotState

class RobotManager():
    def __init__(self):
        self.robots = {}

    def register(self, id):
        if id not in self.robots:
            self.robots[id] = RobotState(id)

    def update_robot(self, id, status=None, battery=None):
        if id not in self.robots:
            self.register(id)

        self.robots[id].update(status, battery)

    def update_location(self, id, x, y):
        if id in self.robots:
            if self.robots[id].online:
                self.robots[id].update_location(x, y)

    def get_robot(self, id):
        return self.robots.get(id)
    
    def get_all_robots(self):
        return sorted(
            self.robots.values(),
            key=lambda robot: robot.id,
        )
    
    def get_disconnected_robot(self, timeout_sec):
        return [
            id for id, robot in self.robots.items() if robot.is_timed_out(timeout_sec)
        ]