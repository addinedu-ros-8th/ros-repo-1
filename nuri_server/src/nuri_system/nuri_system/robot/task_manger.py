from collections import deque

class TaskManager():
    def __init__(self, robot_manager):
        self.robot_manager = robot_manager
        self.task_list = deque()

    def assign_task(self):
        robots = self.robot_manager.get_all_robots()

        for robot in robots:
            if robots.status in ["대기", "충전"]: