from setuptools import setup
import os
from glob import glob

package_name = 'speedy_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models', 'obstacle_detector'),
         glob('models/obstacle_detector/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nycocado',
    maintainer_email='nycolascanutto@gmail.com',
    description='Camera-based line and obstacle detection.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'line_detector = speedy_vision.line_detector_node:main',
            'obstacle_detector = speedy_vision.obstacle_detector_node:main',
        ],
    },
)
