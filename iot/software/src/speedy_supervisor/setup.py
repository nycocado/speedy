from setuptools import find_packages, setup

package_name = 'speedy_supervisor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nycocado',
    maintainer_email='nycolascanutto@gmail.com',
    description='State machine and hardware multiplexer supervisor',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'speedy_supervisor = speedy_supervisor.speedy_supervisor:main'
        ],
    },
)