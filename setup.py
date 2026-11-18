from glob import glob

from setuptools import find_packages, setup

package_name = 'pinky_sim_examples'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/layouts', glob('layouts/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*.pgm') + glob('maps/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mentor',
    maintainer_email='mentor@example.com',
    description='Pinky Pro Gazebo 실습 예제',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_driver = pinky_sim_examples.command_driver:main',
            'make_world = pinky_sim_examples.world_builder:main',
            'make_map = pinky_sim_examples.map_builder:main',
            'nav2_patrol = pinky_sim_examples.nav2_patrol:main',
            'pose_error = pinky_sim_examples.pose_error:main',
            'spawn_object = pinky_sim_examples.spawn_object:main',
            'send = pinky_sim_examples.send_command:main',
            'sim_devices = pinky_sim_examples.sim_devices:main',
            'layout_markers = pinky_sim_examples.layout_markers:main',
            'us_range = pinky_sim_examples.us_range:main',
        ],
    },
)
