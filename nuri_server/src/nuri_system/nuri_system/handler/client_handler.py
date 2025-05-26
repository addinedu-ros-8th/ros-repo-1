import os
import cv2
import datetime
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory

from nuri_system.database.datbase_connection import NuriDatabase
from nuri_system.handler.opcode import Opcode
from nuri_system.network.packet.client_packet import ClientPacket

class ClientHandler():
    def handle_packet(handler, reader, node, robot_handler):
        opcode = reader.read_byte()

        # print(f"[OPCODE] {Opcode(opcode).name}")
        # node.get_logger().info(f"{Opcode(opcode).name}")

        if opcode == Opcode.CLIENT_HELLO.value:     # Client Register
            ClientHandler.client_hello(handler, reader, node)
        if opcode == Opcode.RESIDENT_LIST.value:
            ClientHandler.fetch_resident_list(handler, reader, node)
        elif opcode == Opcode.SEND_RESIDENT_INFO.value:
            ClientHandler.add_new_resident(handler, reader, node)
        elif opcode == Opcode.REQUEST_RESIDENT_INFO.value:
            ClientHandler.fetch_resident_info(handler, reader, node)
        elif opcode == Opcode.REQUEST_DISCHARGE.value:
            ClientHandler.discharge_resident(handler, reader)
        elif opcode == Opcode.UPDATE_RESIDENT_INFO.value:
            ClientHandler.update_resident_info(handler, reader)
        elif opcode == Opcode.DELETE_RESIDENT.value:
            ClientHandler.delete_resident_info(handler, reader)
        elif opcode == Opcode.RESIDENT_NAME_LIST.value:
            ClientHandler.fetch_resident_name_list(handler, reader)
        elif opcode == Opcode.LOCATION_LIST.value:
            ClientHandler.fetch_location_list(handler, reader, node)
        elif opcode == Opcode.BED_LIST.value:
            ClientHandler.fetch_bed_count(handler, reader, node)
        elif opcode == Opcode.RESIDENT_HEALTH_INFO.value:
            ClientHandler.update_health_info(handler, reader, node)
        elif opcode == Opcode.PATROL_LIST.value:
            ClientHandler.fetch_patrol_schedule(handler, reader, node)
        elif opcode == Opcode.PATROL_REGIST.value:
            ClientHandler.regist_patrol(handler, reader, node)
        elif opcode == Opcode.PATROL_UNREGIST.value:
            ClientHandler.unregist_patrol(handler, reader, node)
        elif opcode == Opcode.WALK_LIST.value:
            ClientHandler.fetch_walk_schedule(handler, reader, node)
        elif opcode == Opcode.WALK_REGIST.value:
            ClientHandler.regist_walk(handler, reader, node)
        elif opcode == Opcode.WALK_UNREGIST.value:
            ClientHandler.unregist_walk(handler, reader, node)
        elif opcode == Opcode.LOG_CATEGORY.value:
            ClientHandler.fetch_log_category(handler, reader, node)
        elif opcode == Opcode.LOG_SEARCH.value:
            ClientHandler.fetch_log_list(handler, reader, node)
        elif opcode == Opcode.DETECTION.value:
            ClientHandler.detection(handler, reader, node)
        elif opcode == Opcode.REQUEST_VIDEO.value:
            ClientHandler.request_video(handler, reader, node)
        elif opcode == Opcode.ROBOT_LIST.value:
            ClientHandler.fetch_robot_list(handler, reader, node)
        elif opcode == Opcode.GOAL_POSE.value:
            ClientHandler.send_goal_pose(handler, reader, robot_handler)
        elif opcode == Opcode.DISTANCE.value:
            ClientHandler.distance(handler, reader, node)
        elif opcode == 0x99:
            ClientHandler.test(handler, reader, node)

    @staticmethod
    def client_hello(handler, reader, node):
        type = reader.read_string()
        identifier = None

        if type == "server":
            identifier = reader.read_string()
        elif type == "device":
            identifier = reader.read_int()

        handler.client_manager.register(handler, type, identifier)
        node.get_logger().info(f"[CONNECTED] type : {type}, {handler.addr}")

        handler.send(ClientPacket.send_hello())
        if type == "client":
            map_path = os.path.join(get_package_share_directory('nuri_system'), 'config', 'map_info.npz')
            loaded = np.load(map_path)
            grid = loaded['grid']                       # shape: (height, width)[0]
            origin = loaded['origin_position']
            map_info = {
                'resolution' : loaded['resolution'].item(),
                'origin_x' : origin[0],
                'origin_y' : origin[1],
                'width' : float(grid.shape[1]),
                'height' : float(grid.shape[0])
            }

            img = np.zeros((int(map_info['height']), int(map_info['width'])), dtype=np.uint8).copy()
            img[grid == 0] = 255
            img[grid == 100] = 0
            img[grid == -1] = 205
            img = np.fliplr(img)
            _, img = cv2.imencode('.jpg', img)
            handler.send(ClientPacket.send_map(map_info, img, node))

    @staticmethod
    def fetch_resident_list(handler, reader, node):
        conn = NuriDatabase()

        status = 0x00

        name = reader.read_string()
        check = reader.read_bool()
        result = None
        params = ()

        try:
            query = """
            SELECT r.id, r.name, r.birthday, r.sex, l.name, ROUND(AVG(h.temperature), 2) AS avg_temperature 
            FROM residents r, health_info h, location l, bed b
            WHERE h.user_id = r.id
            AND b.id = r.bed_id
            AND b.location_id = l.id
            """
            if name != "":
                query += " AND r.name like %s"
                params = (f"%{name}%",)

            if not check:
                join = " AND " if name == "" else " AND"
                query += join + " discharge_date is null"

            query += " GROUP BY r.id"

            result = conn.fetch_all(query, params)
        except Exception as e:
            status = 0xFF
            node.get_logger().info(f'{e}')


        handler.send(ClientPacket.send_resident_list(status, result))

    @staticmethod
    def add_new_resident(handler, reader, node):
        conn = NuriDatabase()
        name = reader.read_string()
        birthday = reader.read_string()
        sex = reader.read_char()
        room_name = reader.read_string()
        bed_number = reader.read_short()
        face = reader.read_image()

        status = 0x00

        try:
            query = "SELECT * FROM residents WHERE name = %s AND birthday = %s"
            result = conn.fetch_one(query, (name, birthday))

            if result:
                status = 0x01
            else:
                query = "INSERT INTO residents(name, birthday, sex, bed_id) values(%s, %s, %s, (SELECT b.id FROM bed b, location l WHERE l.name = %s AND l.id = b.location_id AND b.bed_index = %s))"
                resident_id = conn.execute_query(query, (name, birthday, sex, room_name, bed_number))

                query = "INSERT INTO faces(resident_id, face_image) values(%s, %s)"
                conn.execute_query(query, (resident_id, face))

                conn.commit()
        except Exception as e:
            status = 0xFF
            conn.rollback()

        handler.send(ClientPacket.send_resident_info_result(status))

    @staticmethod
    def fetch_resident_info(handler, reader, node):
        conn = NuriDatabase()

        name = reader.read_string()
        birthday = reader.read_string()

        status = 0x00
        result = None

        try:
            query = """
                    SELECT r.name, r.sex, r.birthday, l.name, b.bed_index, ROUND(AVG(h.temperature), 2) AS avg_temperature, f.face_image
                    FROM residents r, faces f, location l, bed b, health_info h
                    WHERE r.name = %s 
                    AND r.birthday = %s 
                    AND r.id = f.resident_id
                    AND l.id = b.location_id
                    AND r.bed_id = b.id AND r.id = h.user_id
                    AND DATE(h.created_at) = CURDATE()
                    """
            result = conn.fetch_one(query, (name, birthday))

        except Exception as e:
            status = 0xFF
            node.get_logger().info(f'{e}')

        handler.send(ClientPacket.send_resident_info(status, result))

    @staticmethod
    def discharge_resident(handler, reader):
        conn = NuriDatabase()
        name = reader.read_string()
        birthday = reader.read_string()
        date = datetime.datetime.now()

        status = 0x00

        try:
            query = "UPDATE residents SET discharge_date = %s WHERE name = %s and birthday = %s"
            conn.execute_query(query, (date, name, birthday))
        except:
            status = 0xFF
            conn.rollback()

        handler.send(ClientPacket.discharge_result(status, name))

    @staticmethod
    def update_resident_info(handler, reader):
        conn = NuriDatabase()
        name = reader.read_string()
        birthday = reader.read_string()
        sex = reader.read_char()
        bed = reader.read_int()
        room = reader.read_int()

        status = 0x00

        try:
            query = "UPDATE residents SET sex = %s WHERE name = %s and birthday = %s"
            conn.execute_query(query, (sex, name, birthday))
        except:
            status = 0xFF
            conn.rollback()

        handler.send(ClientPacket.update_resident_info_result(status))

    @staticmethod
    def delete_resident_info(handler, reader):
        conn = NuriDatabase()
        name = reader.read_string()
        birthday = reader.read_string()

        status = 0x00
        
        try:
            query = "DELETE FROM residents WHERE name = %s and birthday = %s"
            conn.execute_query(query, (name, birthday))
        except:
            status = 0xFF
            conn.rollback()

        handler.send(ClientPacket.delete_resident_info_result(status))

    @staticmethod
    def fetch_resident_name_list(handler, reader):
        conn = NuriDatabase()

        status = 0x00
        result = None

        try:
            query = "SELECT name FROM residents"
            result = conn.fetch_all(query)
        except:
            status = 0xFF

        handler.send(ClientPacket.send_resident_name_list(status, result))

    @staticmethod
    def fetch_location_list(handler, reader, node):
        conn = NuriDatabase()

        status = 0x00
        result = None

        try:
            query = "SELECT name FROM location l"
            result = conn.fetch_all(query)
        except:
            status = 0xFF

        handler.send(ClientPacket.send_location_list(status, result))

    @staticmethod
    def fetch_bed_count(handler, reader, node):
        conn = NuriDatabase()
        location = reader.read_string()

        status = 0x00
        result = None

        try:
            query = "SELECT count(*) FROM location l, bed b WHERE l.id = b.location_id AND l.name = %s"
            result = conn.fetch_one(query, (location,))
        except Exception as e:
            status = 0xFF

        handler.send(ClientPacket.send_bed_count(status, result))

    @staticmethod
    def update_health_info(handler, reader, node):
        conn = NuriDatabase()
        resident_id = reader.read_byte()
        heart_rate = reader.read_int()
        oxygen = reader.read_int()
        temperature = reader.read_float()

        try:
            query = "INSERT INTO health_info(user_id, heart_rate, oxygen, temperature) VALUES(%s, %s, %s, %s)"
            conn.execute_query(query, (resident_id, heart_rate, oxygen, temperature))
        except:
            conn.rollback()

    @staticmethod
    def fetch_log_category(handler, reader, node):
        conn = NuriDatabase()

        status = 0x00

        try:
            query = "SELECT name FROM job_category"
            type = conn.fetch_all(query)
            type = [row[0] for row in type]

            query = "SELECT name FROM robot"
            name = conn.fetch_all(query)
            name = [row[0] for row in name]
        except:
            conn.rollback()
            status = 0xFF

        handler.send(ClientPacket.send_log_category(status, type, name))

    @staticmethod
    def fetch_log_list(handler, reader, node):
        conn = NuriDatabase()
        start = reader.read_string()
        end = reader.read_string()
        event_type = reader.read_string()
        robot = reader.read_byte()
        keyword = reader.read_string()

        status = 0x00
        result = None

        param = [start, end]

        try:
            query = """
            SELECT j.name, l.robot_id, l.comment, l.created_at 
            FROM logs l, job_category j 
            WHERE l.job_id = j.id and (l.created_at >= %s or l.created_at <= %s)
            """
            if event_type != "전체":
                query += " AND j.name = %s "
                param.append(event_type)
            if robot != 0:
                query += " AND l.robot_id = %s "
                param.append(robot)
            if keyword != "":
                query += " AND l.comment LIKE '%s' "
                keyword_param = f"%{keyword}%"
                param.append(keyword_param)

            result = conn.fetch_all(query, tuple(param))
        except Exception as e:
            node.get_logger().info(f"{e}")
            status = 0xFF

        handler.send(ClientPacket.send_log_list(status, result))

    @staticmethod
    def detection(handler, reader, node):
        conn = NuriDatabase()

        robot_id = reader.read_byte()
        type = reader.read_string()

        node.get_logger().info(f'{type}')

        packet = ClientPacket.send_detection(robot_id, type)

        handler.client_manager.broadcast(packet)

        # robots = handler.robot_handler.robot_manager.get_all_robots()
        # handler.client_manager.broadcast(ClientPacket.send_robot_list(robots))

    @staticmethod
    def fetch_robot_list(handler, reader, node):
        robots = handler.robot_handler.robot_manager.get_all_robots()

        handler.send(ClientPacket.send_robot_list(robots))

    @staticmethod
    def request_video(handler, reader, node):
        addr = reader.read_string()
        robot_id = reader.read_byte()

        pub = node.create_publisher(String, f'/robot{robot_id}/emergency_msg', 10)
        msg = String()
        msg.data = addr
        pub.publish(msg)

        print(msg, flush=True)

    @staticmethod
    def fetch_patrol_schedule(handler, reader, node):
        conn = NuriDatabase()

        status = 0x00
        result = None

        try:
            query = "SELECT scheduled_time FROM schedule WHERE job_id = 4 order by scheduled_time ASC"
            result = conn.fetch_all(query)
        except:
            status = 0xFF

        handler.send(ClientPacket.send_patrol_schedule(status, result))

    @staticmethod
    def regist_patrol(handler, reader, node):
        conn = NuriDatabase()
        time = reader.read_string()

        status = 0x00

        try:
            query = "INSERT INTO schedule(job_id, scheduled_time) VALUES(4, %s)"
            conn.execute_query(query, (time,))
        except:
            conn.rollback()
            status = 0xFF

        handler.send(ClientPacket.regist_patrol_result(status))
        handler.robot_handler.update_schedule_time()

    @staticmethod
    def unregist_patrol(handler, reader, node):
        conn = NuriDatabase()
        time = reader.read_string()

        status = 0x00

        try:
            query = "DELETE FROM schedule WHERE scheduled_time = %s"
            conn.execute_query(query, (time,))
        except:
            conn.rollback()
            status = 0xFF

        handler.send(ClientPacket.unregist_patrol_result(status))

        handler.robot_handler.update_schedule_time()

    @staticmethod
    def fetch_walk_schedule(handler, reader, node):
        conn = NuriDatabase()

        status = 0x00
        result = None

        try:
            query = """
                SELECT r.name, DATE_FORMAT(s.scheduled_time, '%H:%i')
                FROM residents r, schedule s, walk_schedule ws 
                WHERE s.id = ws.schedule_id 
                AND r.id = ws.user_id 
                ORDER BY s.scheduled_time ASC
            """
            result = conn.fetch_all(query)
        except Exception as e:
            status = 0xFF
            node.get_logger().info(f'{e}')

        handler.send(ClientPacket.send_walk_schedule(status, result))

    @staticmethod
    def regist_walk(handler, reader, node):
        conn = NuriDatabase()
        name = reader.read_string()
        time = reader.read_string()

        status = 0x00

        try:
            query = "INSERT INTO schedule(job_id, scheduled_time) VALUES(2, %s)"
            id = conn.execute_query(query, (time,))

            query = "INSERT INTO walk_schedule(schedule_id, user_id) VALUES(%s, (SELECT id FROM residents WHERE name = %s))"
            conn.execute_query(query, (id, name))
        except Exception as e:
            conn.rollback()
            status = 0xFF
            node.get_logger().info(f'{e}')

        handler.send(ClientPacket.regist_walk_result(status))
        handler.robot_handler.update_schedule_time()

    @staticmethod
    def unregist_walk(handler, reader, node):
        conn = NuriDatabase()
        name = reader.read_string()
        time = reader.read_string()

        status = 0x00

        try:
            query = """
                DELETE FROM schedule 
                WHERE id = (
                    SELECT w.schedule_id 
                    FROM walk_schedule w, residents r, schedule s 
                    WHERE r.id = w.user_id 
                    AND r.name = %s 
                    AND s.scheduled_time = %s 
                    AND w.schedule_id = s.id
                )
            """
            conn.execute_query(query, (name, time))
        except Exception as e:
            status = 0xFF
            conn.rollback()
            node.get_logger().info(f'{e}')

        handler.send(ClientPacket.unregist_walk_result(status))
        handler.robot_handler.update_schedule_time()

    @staticmethod
    def send_goal_pose(handler, reader, robot_handler):
        goal_x = reader.read_float()
        goal_y = reader.read_float()
        robot_id = reader.read_byte()

        # robot_handler.task_manager.send_goal_pose(robot_id, goal_x, goal_y)

    @staticmethod
    def distance(handler, reader, node):
        robot_id = reader.read_byte()
        distance = reader.read_string()
        
        pub = node.create_publisher(String, f'/robot{robot_id}/report_closefar', 10)
        msg = String()
        msg.data = distance
        pub.publish(msg)

        node.get_logger().info(f'{robot_id} {distance}')

    @staticmethod
    def test(handler, reader, node):
        # client_manager = SocketHandler.client_manager
        # client_manager.broadcast(ClientPacket.test(), "server")

        pub = node.create_publisher(String, '/arrive', 10)
        msg = String()
        msg.data = "황한문"
        pub.publish(msg)