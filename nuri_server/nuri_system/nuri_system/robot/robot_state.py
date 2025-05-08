import time

class RobotState():
    def __init__(self, id):
        self.id = id
        self.battery = -1
        self.status = "StandBy"
        self.last_update = 0.0
        self.online = False

    def update(self, status, battery):
        self.status = status
        self.battery = battery
        self.online = True
        self.last_update = time.time()

    def get(self):
        return {
            "id" : self.id,
            'status' : self.status,
            'battery' : self.battery,
            'online' : self.online,
            'last_update' : self.last_update
        }
    
    def is_timed_out(self, timeout_sec):
        return (time.time() - self.last_update) > timeout_sec