from glob import glob
from setuptools import setup
import os

package_name = 'speedy_vision_pc'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models/obstacle_detector'),
         glob('models/obstacle_detector/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nycocado',
    maintainer_email='nycolascanutto@gmail.com',
    description='YOLO Obstacle Detection offloaded to PC.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'obstacle_detector = speedy_vision_pc.obstacle_detector_node:main',
        ],
    },
)
