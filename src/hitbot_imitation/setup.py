"""Package setup for hitbot_imitation (ament_python)."""
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'hitbot_imitation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raushan',
    maintainer_email='raushan@xprobotics.ai',
    description='"Do as I Do" human-video imitation pipeline for the HITBOT '
                'Z-Arm S922 (6-DOF), executed in Gazebo via MoveIt.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Stage A (needs perception deps installed on the GPU machine)
            'reconstruct = hitbot_imitation.app_reconstruct:main',
            # Stage B (pure numpy/scipy)
            'retarget = hitbot_imitation.app_retarget:main',
            # Stage C (ROS 2 + MoveIt)
            'execute_moveit = hitbot_imitation.app_execute:main',
        ],
    },
)
