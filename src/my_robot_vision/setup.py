from setuptools import find_packages, setup

package_name = 'my_robot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ultralytics', 'torch', 'torchvision'],
    zip_safe=True,
    maintainer='wb',
    maintainer_email='1878087979@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apple_detector = my_robot_vision.apple_detector:main',
        ],
    },
)
