from setuptools import find_packages, setup

package_name = 'ai_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='hhm9124@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_server_node = ai_server.udp_server_node:main',
            'tcp_server_node = ai_server.tcp_server_node:main',
            'ros_server_node = ai_server.ros_server_node:main',
        ],
    },
)
