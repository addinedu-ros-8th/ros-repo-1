from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = get_package_share_directory('nuri_system')
    bridge1_yaml = os.path.join(config_dir, 'config', 'bridge_1.yaml')
    bridge2_yaml = os.path.join(config_dir, 'config', 'bridge_2.yaml')

    robot_domains = [1, 2]

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
                # name=f'domain_bridge_{domain}',
                # parameters=[{'yaml_config': os.path.join(path, f'bridge_{domain}.yaml')}]
                arguments=[path]
            )
        )

    return LaunchDescription(node)