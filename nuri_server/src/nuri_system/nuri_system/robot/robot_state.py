import time
from rclpy.node import Node

from nuri_system.database.datbase_connection import NuriDatabase

class RobotState():
    def __init__(self, id):
        self.id = id
        self.battery = -1
        self.status = "대기"
        self.last_update = 0.0
        self.online = False
        self.x = -1
        self.y = -1

        self.q_x = 0
        self.q_y = 0
        self.q_z = 0
        self.q_w = 0

    def update(self, status=None, battery=None):
        if status is not None:
            self.update_status(status)

            conn = NuriDatabase.get_instance()
            try:
                query = "UPDATE robot SET status_id = (SELECT name FROM robot_status WHERE type = %s) WHERE id = %s"
                conn.execute_query(query, (status, self.id))
            except:
                conn.rollback()

        if battery is not None:
            self.update_battery(battery)

        if not self.online:
            self.online = True
            
        self.last_update = time.time()

    def update_status(self, status):
        self.status = status

    def update_battery(self, battery):
        self.battery = battery

    def update_location(self, position, orientation):
        self.x = position.x
        self.y = position.y
        self.q_x = orientation.x
        self.q_y = orientation.y
        self.q_z = orientation.z
        self.q_w = orientation.w

    def get(self):
        return {
            "id" : self.id,
            'status' : self.status,
            'battery' : self.battery,
            'online' : self.online,
            'x' : self.x,
            'y' : self.y,
            'last_update' : self.last_update
        }
    
    def is_timed_out(self, timeout_sec):
        return (time.time() - self.last_update) > timeout_sec