class Robot():
    def __init__(self, id, name, battery, status='StandBy'):
        self.id = id
        self.name = name
        self.status = status
        self.battery = battery