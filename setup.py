from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robocar_visual_pursuit_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.blob'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='ROS2 package for RC car visual pursuit using DepthAI and YOLO',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'parameter_tuner_node = robocar_visual_pursuit_pkg.parameter_tuner_node:main',
            'car_detection_node = robocar_visual_pursuit_pkg.car_detection_node:main',
            'lane_guidance_node = robocar_visual_pursuit_pkg.lane_guidance_node:main',
            'led_controller_node = robocar_visual_pursuit_pkg.led_controller_node:main',
        ],
    },
)
