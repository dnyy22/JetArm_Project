import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 1. 註冊 package 到 ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # 2. 安裝 package.xml
        ('share/' + package_name, ['package.xml']),
        
        # 3. 安裝 Launch 檔案 (如果有 launch 資料夾或直接放在套件內)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 4. 安裝 TensorRT 模型與 C++ 外掛庫檔案至 share/my_arm_control/models (方便程式動態讀取)
        ('share/' + package_name + '/models', [
            package_name + '/best.engine',
            package_name + '/libmyplugins.so'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='zouzhaotao@gmail.com',
    description='Weld tracking and YOLOv8 TensorRT control package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'weld_track = my_arm_control.weld_track:main',
            'yolov8_node = my_arm_control.yolov8_node:main',
        ],
    },
)
