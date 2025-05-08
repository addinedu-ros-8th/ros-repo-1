class RobotHandler:
    def __init__(self):
        self.robots = {}

    def add_robot(self, robot_id, robot_name, robot_status, domain_id):
        self.robots[robot_id] = {
            'name' : robot_name,
            'status' : robot_status,
            'domain_id' : domain_id
        }

    def update_battery(self, robot_id, battery):
        if robot_id in self.robots:
            self.robots[robot_id]['battry'] = battery

    def update_status(self, robot_id, status_id):
        if robot_id in self.robots:
            if status_id == 1:
                status = "대기중"
            elif status_id == 2:
                status = "작업중"
            elif status_id == 3:
                status = "응급상황"
            elif status_id == 4:
                status = "충전중"

            self.robots[robot_id]['status'] = status