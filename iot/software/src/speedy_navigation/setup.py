from setuptools import setup

package_name = 'speedy_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nycocado',
    maintainer_email='nycolascanutto@gmail.com',
    description='Controlador reativo: segue linha + desvia de paredes -> bicycle_steering_controller.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'reactive_controller = speedy_navigation.reactive_controller_node:main',
        ],
    },
)
