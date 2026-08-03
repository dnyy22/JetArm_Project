"""Launch the weld-tracking stack with portable package-relative paths."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    engine_path = LaunchConfiguration('engine_path')
    plugin_path = LaunchConfiguration('plugin_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'engine_path',
            default_value=PathJoinSubstitution([FindPackageShare('my_arm_control'), 'models', 'best.engine']),
        ),
        DeclareLaunchArgument(
            'plugin_path',
            default_value=PathJoinSubstitution([FindPackageShare('my_arm_control'), 'models', 'libmyplugins.so']),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('servo_controller'), 'launch', 'servo_controller.launch.py',
        ]))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('depth_cam'), 'launch', 'depth_cam.launch.py',
        ]))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('kinematics'), 'launch', 'kinematics_node.launch.py',
        ]))),
        Node(
            package='my_arm_control',
            executable='yolov8_node',
            name='yolov8',
            output='screen',
            parameters=[{
                'classes': ['weldboard_s', 'weldboard_l', 'al_plane_s', 'al_plane_m', 'al_plane_l'],
                'engine': engine_path,
                'lib': plugin_path,
                'conf': 0.8,
                'start': False,
            }],
        ),
        Node(
            package='my_arm_control',
            executable='weld_track',
            name='yolo_touch_planner',
            output='screen',
        ),
    ])
