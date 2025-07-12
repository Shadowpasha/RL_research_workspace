#!/usr/bin/env python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

param_config = os.path.join(get_package_share_directory('depthimage_to_laserscan'), 'cfg', 'param.yaml')
def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    return LaunchDescription([
        # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
        # Node(
        #     package='px4_offboard',
        #     namespace='px4_offboard',
        #     executable='visualizer',
        #     name='visualizer'
        # ),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --'
        ),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            remappings=[('depth','/camera/depth/image_raw'),
                        ('depth_camera_info', '/camera/depth/camera_info'),
                        ('scan', '/realsense_scan_top')],
            parameters=[{
                "scan_height": 30,
                "scan_time": 0.033,
                "range_min": 0.3,
                "range_max": 5.0,
                "output_frame": "base_link",
                "offset": 50
        }]),

        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            remappings=[('depth','/camera/depth/image_raw'),
                        ('depth_camera_info', '/camera/depth/camera_info'),
                        ('scan', '/realsense_scan')],
            parameters=[{
                "scan_height": 30,
                "scan_time": 0.033,
                "range_min": 0.3,
                "range_max": 5.0,
                "output_frame": "base_link",
                "offset": 0
        }]),

        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            remappings=[('depth','/camera/depth/image_raw'),
                        ('depth_camera_info', '/camera/depth/camera_info'),
                        ('scan', '/realsense_scan_bottom')],
            parameters=[{
                "scan_height": 30,
                "scan_time": 0.033,
                "range_min": 0.3,
                "range_max": 5.0,
                "output_frame": "base_link",
                "offset": -50
        }]),

        # Node(
        #     package="laser_filters",
        #     executable="scan_to_scan_filter_chain",
        #     parameters=[
        #         PathJoinSubstitution([
        #             get_package_share_directory("px4_offboard"),
        #             "launch", "box_filter.yaml",
        #         ])],
        # ),

        # Node(
        #     package='drone_pose',
        #     executable='DronePoseExecutable',
        #     name='drone_pose',
        #     output={
        #             "stdout": "screen",
        #             "stderr": "screen",
        #     },
        #     parameters=[{
        #         'base_frame':'map',
        #         "child_frame": 'base_link',
        #         "pub.pose": True,
        #         "pub.vel": True,
        #         "pub.path": True,

        #     }],
        # ),

        #  Node(
        #     package='mavros',
        #     executable='mavros_node',
        #     name='mavros_node',
        #     output={
        #             "stdout": "screen",
        #             "stderr": "screen",
        #     },
        #     parameters=[{
        #         'fcu_url':'udp://:14540@localhost:14557',
        #     }],
        # ),
        # Node(package = "tf2_ros", 
        #                executable = "static_transform_publisher",
        #                arguments = ["0.1", "0", "0", "0.0", "0", "0.0", "base_link", "depth_camera::camera_optical_link"]),
        # Node(
        #     package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        #     remappings=[('cloud_in', '/camera/points'),
        #                 ('scan', '/scan')],
        #     parameters=[{
        #         # 'target_frame': 'base_link',
        #         # 'transform_tolerance': 0.2,
        #         'min_height': -0.2,
        #         'max_height': 0.2,
        #         'angle_min': -0.755,  # -M_PI/2
        #         'angle_max': 0.755,  # M_PI/2
        #         'angle_increment': 0.016,  # M_PI/360.0
        #         'scan_time': 0.05,
        #         'range_min': 0.3,
        #         'range_max': 4.0,
        #         'use_inf': False,
        #         'inf_epsilon': 1.0
        #     }],
        #     name='pointcloud_to_laserscan'
        # ),
        # Node(
        #     package='px4_offboard',
        #     namespace='px4_offboard',
        #     executable='control',
        #     name='control',
        #     prefix='gnome-terminal --',
        # ),
        # IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource([
        #             FindPackageShare("stereo_image_proc"), '/launch', '/stereo_image_proc.launch.py'])
        # )
        # ,
        # Node(
        #     package='px4_offboard',
        #     namespace='px4_offboard',
        #     executable='velocity_control',
        #     name='velocity',
        # ),
        # Node(
        #     package='rviz2',
        #     namespace='',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        # )
    ])
