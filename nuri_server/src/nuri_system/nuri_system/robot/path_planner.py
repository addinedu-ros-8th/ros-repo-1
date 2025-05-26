import os
import numpy as np
import rclpy
from queue import PriorityQueue
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

class PathPlanner(Node):
    def __init__(self, robot_manager):
        super().__init__('path_planner_node')

        self.robot_manager = robot_manager
        
        self.path_plan_publisher = {}

        self.map_load()

    def map_load(self):
        loaded = np.load(os.path.join(get_package_share_directory('nuri_system'), 'config', 'map_info.npz'))
        self.grid = loaded['grid']                       # shape: (height, width)[0]
        origin = loaded['origin_position']
        self.map_info = {
            'resolution' : loaded['resolution'].item(),
            'origin_x' : origin[0],
            'origin_y' : origin[1],
            'width' : self.grid.shape[1],
            'height' : self.grid.shape[0]
        }

    def init_path_publisher(self):
        for robot in self.robot_manager.get_all_robots():
            topic = f'/robot{robot.id}/path_plan'
            self.path_plan_publisher[robot.id] = self.create_publisher(PathPlan, topic, 10)

    def world_to_map(self, point):
        mx = int((point[0] - self.map_info['origin_x']) / self.map_info['resolution'])
        my = int((point[1] - self.map_info['origin_y']) / self.map_info['resolution'])
        return (mx, my)
    
    def map_to_world(self, point):
        x = point[0] * self.map_info['resolution'] + self.map_info['origin_x'] + self.map_info['resolution'] / 2.0
        y = point[1] * self.map_info['resolution'] + self.map_info['origin_y'] + self.map_info['resolution'] / 2.0
        return (x, y)


    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))
    
    def a_star(self, start, goal):
        path = self.a_star2(start, goal)

        return [self.map_to_world(r, c) for r, c in path]

    def a_star2(self, start, goal):
        start = self.world_to_map(start)
        goal = self.world_to_map(goal)

        rows, cols = self.grid.shape
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}

        while not open_set.empty():
            _, current = open_set.get()

            if current == goal:
                path = []
                visited = set()
                while current in came_from:
                    if current in visited:  # 무한 루프 방지
                        break
                    visited.add(current)

                    current = came_from[current]
                    path.append(current)

                return path[::-1]


            neighbors = []
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = current[0] + dx, current[1] + dy
                if 0 <= nx < rows and 0 <= ny < cols and self.grid[nx, ny] == 0:
                    neighbors.append((nx, ny))

            for neighbor in neighbors:
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    open_set.put((f_score, neighbor))

        return []  # No path found