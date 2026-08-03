import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, OpaqueFunction


def launch_setup(context):
    # 1. 根據是否編譯 (need_compile) 動態切換 Package 路徑
    compiled = os.environ.get('need_compile', 'False')

    if compiled == 'True':
        servo_pkg_path = get_package_share_directory('servo_controller')
        depth_cam_pkg_path = get_package_share_directory('depth_cam')
        example_pkg_path = get_package_share_directory('example')
        kinematics_pkg_path = get_package_share_directory('kinematics')
    else:
        # 開發/未編譯模式下使用絕對路徑
        servo_pkg_path = '/home/ubuntu/ros2_ws/src/driver/servo_controller'
        peripherals_pkg_path = '/home/ubuntu/ros2_ws/src/peripherals'
        example_pkg_path = '/home/ubuntu/ros2_ws/src/example'
        kinematics_pkg_path = '/home/ubuntu/ros2_ws/src/driver/kinematics'

    # 2. 宣告子 Launch 檔 (使用 IncludeLaunchDescription 載入各模組的 launch.py)
    # (A) 馬達驅動 Launch
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(servo_pkg_path, 'launch', 'servo_controller.launch.py')
        )
    )

    # (B) 深度相機驅動 Launch
    depth_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_pkg_path, 'launch', 'depth_cam.launch.py')
        )
    )

    # (C) 逆運動學 (IK) 服務 Launch
    kinematics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kinematics_pkg_path, 'launch', 'kinematics_node.launch.py')
        )
    )

    # 3. 直接宣告個別節點 (例如你範例中的 YOLO 節點)
    yolov8_node = Node(
        package='example',
        executable='yolov8_node',
        output='screen',
        parameters=[
            {'classes': ['weldboard_s', 'weldboard_l', 'al_plane_s', 'al_plane_m', 'al_plane_l']},
            {'use_depth': True, 'engine': 'best.engine', 'lib': 'libmyplugins.so', 'conf': 0.8}
        ]
    )

    # 4. 傳回所有需要被一併啟動的 Launch 與 Node 清單
    return [
        servo_launch,
        depth_cam_launch,
        kinematics_launch,
        yolov8_node
    ]


def generate_launch_description():
    """ROS 2 標準 Launch 入口點"""
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    # 支援直接執行 python3 system_all.launch.py
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()