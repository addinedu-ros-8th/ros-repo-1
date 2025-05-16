
import yaml, os
import configparser
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = get_package_share_directory('nuri_system')
    config_path = os.path.join(get_package_share_directory('nuri_system'), 'config', 'config.ini')
    config = configparser.ConfigParser()
    config.read(config_path)

    for filename in os.listdir(os.path.dirname(config_path)):
        file_path = os.path.join(os.path.dirname(config_path), filename)
        if os.path.isfile(file_path) and ".yaml" in filename:
            os.remove(file_path)

    robot_domains = [int(x.strip()) for x in config['domain']['robot_id'].split(',')]
    for domain in robot_domains:
        generate_bridge_yaml(domain, int(config['domain']['server_id']), os.path.dirname(config_path))

    node = [
        Node(
            package='nuri_system',
            executable='main_server',
            name='main_server_node',
            output='screen'
        )
    ]

    for domain in robot_domains:
        path = os.path.join(config_dir, 'config', f'bridge_{domain}.yaml')
        node.append(
            Node(
                package='domain_bridge',
                executable='domain_bridge',
                arguments=[path]
            )
        )

    return LaunchDescription(node)

def generate_bridge_yaml(domain_id, server_id, output_path):
    # topic 구성
    topics = {
        # 로봇 -> 서버
        'state': {
            'type': 'nuri_msgs/msg/NuriBotState',
            'remap': f'/robot{domain_id}/state'
        },
        'tracked_pose': {
            'type': 'geometry_msgs/msg/PoseStamped',
            'remap': f'/robot{domain_id}/tracked_pose'
        },
        'map': {
            'type': 'nav_msgs/msg/OccupancyGrid',
            'remap': f'/robot{domain_id}/map'
        },

        # 서버 -> 로봇
        f'/robot{domain_id}/emergency_msg': {
            'from_domain': server_id,
            'to_domain': domain_id,
            'type': 'std_msgs/msg/String',
            'remap': '/emergency_msg'
        },
        f'/robot{domain_id}/status_list': {
            'from_domain': server_id,
            'to_domain': domain_id,
            'type': 'std_msgs/msg/String',
            'remap': '/status_list'
        },
        f'/robot{domain_id}/schedule': {
            'from_domain': server_id,
            'to_domain': domain_id,
            'type': 'nuri_msgs/msg/NuriBotSchedule',
            'remap': '/schedule'
        },
        f'/robot{domain_id}/patrol': {
            'from_domain': server_id,
            'to_domain': domain_id,
            'type': 'std_msgs/msg/Bool',
            'remap': '/patrol'
        },
        f'/robot{domain_id}/goal_pose': {
            'from_domain': server_id,
            'to_domain': domain_id,
            'type': 'geometry_msgs/msg/PoseStamped',
            'remap': '/goal_pose'
        }
    }

    bridge_data = {
        'from_domain': domain_id,
        'to_domain': server_id,
        'topics': topics
    }

    with open(os.path.join(output_path, f'bridge_{domain_id}.yaml'), 'w') as f:
        yaml.dump(bridge_data, f, sort_keys=False)

    return output_path