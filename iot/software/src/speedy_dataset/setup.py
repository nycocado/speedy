from setuptools import setup
import os
from glob import glob

package_name = 'speedy_dataset'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nycocado',
    maintainer_email='nycolascanutto@gmail.com',
    description="Photo collector for training the Speedy robot's AI model.",
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collector = speedy_dataset.collector_node:main'
        ],
    },
)
