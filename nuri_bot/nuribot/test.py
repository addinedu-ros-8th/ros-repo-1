import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from picamera2 import Picamera2


class ArucoFollowerNode(Node):
    def __init__(self):
        super().__init__('aruco_follower')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 카메라 설정
        self.picam2 = Picamera2()
        self.picam2.configure(
            self.picam2.create_preview_configuration(
                main={"format": 'RGB888', "size": (640, 480)}
            )
        )
        self.picam2.start()

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.camera_matrix = np.array([
            [557.17101254, 0.0, 330.69803327],
            [0.0, 551.10676372, 256.81536916],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([[0.31762515, -1.29886935, -0.00451341,  0.02813533,  2.06417285]])

        self.marker_length = 0.05
        self.distance_threshold = 0.18
        self.center_tolerance = 10

        self.moving = True
        self.shutdown_flag = False  # 🔺 shutdown 요청 여부 플래그
        self.marker_check = False

        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def timer_callback(self):
        frame = self.picam2.capture_array()

        if frame is None or frame.size == 0:
            self.get_logger().error("❌ 비어있는 프레임")
            self.stop_robot()
            self.shutdown_flag = True
            return

        frame = cv2.flip(frame, -1)  # 뒤집기
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        twist = Twist()

        if ids is not None and len(ids) > 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            distance = tvecs[0][0][2]
            self.get_logger().info(f"🎯 마커 인식! 거리: {distance:.3f} m")

            if distance <= self.distance_threshold:
                self.get_logger().info("🛑 도달 거리, 정지 요청.")
                self.stop_robot()
                self.shutdown_flag = True

            img_center_x = frame.shape[1] // 2
            marker_center_x = int(np.mean(corners[0][0][:, 0]))
            offset_x = marker_center_x - img_center_x

            if abs(offset_x) > self.center_tolerance:
                twist.angular.z = -0.003 * offset_x
            else:
                twist.angular.z = 0.0

            twist.linear.x = 0.1
            self.cmd_pub.publish(twist)

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for rvec, tvec in zip(rvecs, tvecs):
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

        elif self.moving:
            twist.linear.x = 0.1
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

        cv2.imshow("Aruco Follower", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            self.get_logger().info("🖱 사용자 ESC, 정지 요청")
            self.stop_robot()
            self.shutdown_flag = True

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.3
        self.cmd_pub.publish(twist)

        time.sleep(1)

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.moving = False


def main(args=None):
    rclpy.init(args=args)
    node = ArucoFollowerNode()

    try:
        while rclpy.ok() and not node.shutdown_flag:
            rclpy.spin_once(node)
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
