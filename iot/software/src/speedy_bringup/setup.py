from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'speedy_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nycocado',
    maintainer_email='nycolascanutto@gmail.com',
    description='Scripts globais de inicialização do sistema (ros2_control RPi PWM, Foxglove, Sensores Base).',
    license='Proprietary',
    entry_points={
        'console_scripts': [
        ],
    },
)
