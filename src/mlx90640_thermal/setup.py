from setuptools import setup
import os
from glob import glob

package_name = 'mlx90640_thermal'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='riyaadh gani',
    maintainer_email='riyaadh.gani2@gmail.com',
    description='MLX90640 human detection ROS2 nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = mlx90640_thermal.publisher_node:main',
            'inference_node = mlx90640_thermal.inference_node:main',
            'thermal_marker_node = mlx90640_thermal.thermal_marker_node:main',
        ],
    },
)
