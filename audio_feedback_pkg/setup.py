from setuptools import setup

package_name = 'audio_feedback_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jingbin JIN',
    maintainer_email='jeongbin0208@g.skku.edu',
    description='Audio feedback for the assistive walker: beeps and English speech (Piper TTS) from fused camera/LiDAR results',
    license='GPL-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_feedback_node = audio_feedback_pkg.audio_feedback_node:main',
            'traffic_light_color_node = audio_feedback_pkg.traffic_light_color_node:main',
            'lidar_distance_speaker_node = audio_feedback_pkg.lidar_distance_speaker_node:main',
            'object_fusion_node = audio_feedback_pkg.object_fusion_node:main',
            'audio_feedback_test = audio_feedback_pkg.audio_feedback_test:main',
            ],
    },
)
