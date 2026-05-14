from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'speedy_teleop'

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
    description='Pacote de interface de controle manual (Teleop) e ponte serial.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'racing_teleop = speedy_teleop.racing_teleop:main'
        ],
    },
)
