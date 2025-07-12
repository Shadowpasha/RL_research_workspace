import os
from glob import glob
from setuptools import setup

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][py]*')),
        (os.path.join('share', package_name), glob('launch/*.ymal')),
        (os.path.join('share', package_name), glob('resource/*rviz'))
        # (os.path.join('share', package_name), ['scripts/TerminatorScript.sh'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Braden',
    maintainer_email='braden@arkelectron.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'offboard_control = px4_offboard.offboard_control:main',
                'visualizer = px4_offboard.visualizer:main',
                'velocity_control = px4_offboard.velocity_control_mavros:main',
                'position_control_guided = px4_offboard.position_control_mavros_guided:main',
                'takeoff_hover = px4_offboard.Takeoff_Hover:main',
                'move_forward = px4_offboard.Move_forward:main',
                'square_path = px4_offboard.Square_Path:main',
                'sending_positions = px4_offboard.sending_positions:main',
                'control = px4_offboard.control:main',
                'processes = px4_offboard.processes:main'
        ],
    },
)
