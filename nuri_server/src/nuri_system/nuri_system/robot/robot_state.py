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
        self.marker_id = 0
        
        self.marker_position = Position()
        self.marker_orientation = Orientation()
        self.position= None
        self.orientation = None

        self.cmd_vel = None

        self.update_marker_id()

        self.drift_count = 0

    def update_marker_id(self):
        conn = NuriDatabase()
        query = "SELECT marker_id FROM robot WHERE domain_id = %s"
        marker_id = conn.fetch_one(query, (self.id,))
        self.marker_id = marker_id

    def update(self, status=None, battery=None):
        if status is not None:
            self.update_status(status)

            # conn = NuriDatabase.get_instance()
            # try:
            #     query = "UPDATE robot SET status_id = (SELECT id FROM robot_status WHERE type = %s) WHERE id = %s"
            #     conn.execute_query(query, (status, self.id))
            # except:
            #     conn.rollback()

        if battery is not None:
            self.update_battery(battery)

        if not self.online:
            self.online = True
            
        self.last_update = time.time()

    def update_cmd_vel(self, msg):
        self.cmd_vel = msg

    def update_status(self, status):
        self.status = status

    def update_battery(self, battery):
        self.battery = battery

    def update_location(self, position, orientation):
        self.position = position
        self.orientation = orientation

    def update_marker_location(self, position):
        self.marker_position.x = position.pose.position
        self.marker_orientation = position.pose.orientation
    
    def is_timed_out(self, timeout_sec):
        return (time.time() - self.last_update) > timeout_sec
    
    def get(self):
        return {
            "id" : self.id,
            "battery" : self.battery,
            "status" : self.status,
            "last_update" : self.last_update,
            "online" : self.online,
            "x" : self.x,
            "y" : self.y,
        }
    
class Position():
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y
        

    def update(self, x, y):
        self.x = x
        self.y = y

    def get(self):
        return {
            "x": self.x,
            "y": self.y
        }
    
class Orientation():
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def update(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def get(self):
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "w": self.w
        }