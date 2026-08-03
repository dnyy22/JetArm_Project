"""Launch only the depth camera and YOLOv8 detector."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start = LaunchConfiguration('start')
    return LaunchDescription([
        DeclareLaunchArgument('start', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('depth_cam'), 'launch', 'depth_cam.launch.py',
        ]))),
        Node(
            package='my_arm_control', executable='yolov8_node', name='yolov8', output='screen',
            parameters=[{
                'classes': ['weldboard_s', 'weldboard_l', 'al_plane_s', 'al_plane_m', 'al_plane_l'],
                'engine': PathJoinSubstitution([FindPackageShare('my_arm_control'), 'models', 'best.engine']),
                'lib': PathJoinSubstitution([FindPackageShare('my_arm_control'), 'models', 'libmyplugins.so']),
                'conf': 0.8,
                'start': start,
            }],
        ),
    ])
