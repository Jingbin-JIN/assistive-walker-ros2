from setuptools import find_packages, setup

package_name = 'lidar_perception_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hyeong-Keun Hong',
    maintainer_email='whaihong@g.skku.edu',
    description='LiDAR perception package of the "ROS 2 기반 자율주행 차량 설계 및 구현" course framework (RPLidar driver, scan processing, obstacle detection)',
    license='GPL-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_publisher_node = lidar_perception_pkg.lidar_publisher_node:main',
			'lidar_processor_node = lidar_perception_pkg.lidar_processor_node:main',
			'lidar_obstacle_detector_node = lidar_perception_pkg.lidar_obstacle_detector_node:main',

        ],
    },
)
