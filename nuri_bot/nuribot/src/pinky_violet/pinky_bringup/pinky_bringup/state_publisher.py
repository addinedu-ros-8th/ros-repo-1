import rclpy
from rclpy.node import Node
from nuri_msgs.msg import NuriBotState
from std_msgs.msg import Float32, Bool, String

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')

        self.state_pub = self.create_publisher(NuriBotState, '/state', 10)
        self.request_pub = self.create_publisher(Bool, '/request_status', 10)

        self.battery_sub = self.create_subscription(Float32, 'pinky_battery_present', self.battery_callback, 10)
        self.response_sub = self.create_subscription(String, '/response_status', self.response_callback, 10)

        self.current_battery = 0.0
        self.prev_battery = 0.0
        self.received_status_list = False
        self.server_status_list = []
        self.status_index = 0

        self.robot_status_list = ['대기', '충전']

        self.timer = self.create_timer(1.0, self.request_status)
        self.create_timer(1.0, self.timer_callback)

    def request_status(self):
        msg = Bool()
        msg.data = True
        self.request_pub.publish(msg)
        self.get_logger().info('Sent request_status True')
        self.timer.cancel()

    def battery_callback(self, msg):
        self.current_battery = msg.data  # float 그대로 저장

    def response_callback(self, msg):
        status_str = msg.data
        filtered = [s.strip() for s in status_str.split(',') if s.strip() not in ['충전', '대기']]
        self.server_status_list = filtered
        self.received_status_list = True
        self.status_index = 0
        self.get_logger().info(f'Received server status list: {self.server_status_list}')

    def timer_callback(self):
        if not self.received_status_list:
            current_status = '대기'
        else:
            battery_diff = self.current_battery - self.prev_battery

            if battery_diff >= 10.0:
                current_status = '충전'
            else:
                combined_list = self.robot_status_list + self.server_status_list
                current_status = combined_list[self.status_index % len(combined_list)]
                self.status_index += 1

            self.prev_battery = self.current_battery

        msg = NuriBotState()
        msg.status = current_status
        msg.battery = float(self.current_battery)  # float로 보내기

        self.state_pub.publish(msg)
        # self.get_logger().info(f'Published: status={msg.status}, battery={msg.battery:.2f}%')

def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
