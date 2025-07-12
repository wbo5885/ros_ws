from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'orb_slam3_ai'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 包含launch文件
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # 包含配置文件
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
        # 包含RViz配置
        (os.path.join('share', package_name, 'rviz'), 
         glob('rviz/*.rviz')),
    ],
    install_requires=[
        'setuptools',
        'torch',
        'torchvision', 
        'opencv-python',
        'numpy',
        'scipy'
    ],
    zip_safe=True,
    maintainer='wb',
    maintainer_email='1878087979@qq.com',
    description='ORB-SLAM3 with AI parameter optimization for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orb_slam3_ai_node = orb_slam3_ai.orb_slam3_ai_node:main',
        ],
    },
)
