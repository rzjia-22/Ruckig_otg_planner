from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'otg_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('otg_planner/launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('otg_planner/config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('otg_planner/urdf/*.xacro')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zoel',
    maintainer_email='zoel@todo.todo',
    description='Joint-space path planning and execution demo for Franka Panda in Gazebo Sim.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'planner = otg_planner.planner_server:main',
            'demo_client = otg_planner.demo_client_node:main',
            'obstacle_scenario = otg_planner.obstacle_scenario_node:main',
        ],
    },
)
