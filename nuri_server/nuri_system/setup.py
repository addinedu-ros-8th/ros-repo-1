from setuptools import find_packages, setup
import os
import glob

package_name = 'nuri_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob.glob(os.path.join('config', '*.yaml'))),
        ('share/' + package_name + '/config', glob.glob(os.path.join('config', 'config.ini'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shin',
    maintainer_email='lt00104@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_server = nuri_system.main_server_node:main'
        ],
    },
)
