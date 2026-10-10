from setuptools import find_packages, setup

package_name = 'fusion_pkg'

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
    maintainer='Jingbin JIN',
    maintainer_email='jeongbin0208@g.skku.edu',
    description='Camera–LiDAR fusion for the assistive walker: announces a detected object when the LiDAR confirms an obstacle ahead',
    license='GPL-3',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'fusion_node = fusion_pkg.fusion_node:main',
        'fusion_speaker_node = fusion_pkg.fusion_speaker_node:main',
        'test_scenario_node = fusion_pkg.test_scenario_node:main',
            ],
    },
)
