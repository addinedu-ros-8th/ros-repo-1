import time

class RobotState():
    def __init__(self, id):
        self.id = id
        self.battery = -1
        self.status = "대기중"
        self.last_update = 0.0
        self.online = False
        self.x = 0
        self.y = 0

    def update(self, status=None, battery=None):
        if status is not None:
            self.update_status(status)
        if battery is not None:
            self.update_battery(battery)
        self.online = True
        self.last_update = time.time()

    def update_status(self, status):
        self.status = status

    def update_battery(self, battery):
        self.battery = battery

    def update_location(self, x, y):
        self.x = x
        self.y = y

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