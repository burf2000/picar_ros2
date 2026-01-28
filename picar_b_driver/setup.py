from setuptools import setup
import os
from glob import glob

package_name = 'picar_b_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='burf2000',
    maintainer_email='burf2000@todo.todo',
    description='ROS2 driver for Adeept PiCar-B robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'picar_node = picar_b_driver.picar_node:main',
            'steering_bridge = picar_b_driver.steering_bridge:main',
        ],
    },
)
