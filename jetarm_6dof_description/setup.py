import os
from glob import glob
from setuptools import setup

package_name = 'jetarm_6dof_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.*'))),
        (os.path.join('share', package_name, 'gazebo'), glob(os.path.join('gazebo', '*.*'))),
        (os.path.join('share', package_name, 'meshes/arm'), glob(os.path.join('meshes/arm', '*.*'))),

        (os.path.join('share', package_name, 'meshes/jetarm_6dof'), glob(os.path.join('meshes/jetarm_6dof', '*.*'))),
        (os.path.join('share', package_name, 'meshes/'), glob(os.path.join('meshes/', '*.STL'))),
        (os.path.join('share', package_name, 'meshes/gripper'), glob(os.path.join('meshes/gripper', '*.*'))),
	(os.path.join('share', package_name, 'meshes/soldering_tool'),
	 glob(os.path.join('meshes/soldering_tool', '*.*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='hiwonder@hiwonder.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
