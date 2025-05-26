import os
import rclpy
import threading

from rclpy.node import Node
from flask import Flask, request, jsonify, render_template
from ament_index_python.packages import get_package_share_directory

from nuri_msgs.srv import NuriBotCall

TEMPLATE_DIR = os.path.join(get_package_share_directory('nuri_system'), 'templates')

app = Flask(__name__, template_folder=TEMPLATE_DIR)
node = None

class CallHandler(Node):
    def __init__(self):
        super().__init__("call_server_node")

        self.get_logger().info(f'[SUCCESS] QR Server Ready')

@app.route('/')
def index():
    id = request.args.get('id')
    return render_template('index.html', id=id)

@app.route('/submit', methods=['POST'])
def submit():
    data = request.get_json()

    call_srv = node.create_client(NuriBotCall, 'call')

    while not call_srv.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("waiting service....")

    request_data = NuriBotCall.Request()
    request_data.id = int(data['id'])
    call_srv.call_async(request_data)
    return jsonify({'status': 'OK'})
    
def run_call_server():
    app.run('0.0.0.0', 9997)

def main():
    global node

    rclpy.init()

    call_thread = threading.Thread(target=run_call_server, daemon=True)
    call_thread.start()

    node = CallHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()