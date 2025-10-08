from setuptools import setup
import os
from glob import glob

package_name = 'onrobot_rg_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takuya Kiyokawa',
    maintainer_email='kiyokawa@hlab.sys.es.osaka-u.ac.jp',
    description='Package to control an OnRobot RG gripper',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'OnRobotRGSimpleController = onrobot_rg_control.OnRobotRGSimpleController:main',
            'OnRobotRGStatusListener = onrobot_rg_control.OnRobotRGStatusListener:main',
            'OnRobotRGTcpNode = onrobot_rg_control.OnRobotRGTcpNode:main',
            'OnRobotRGSimpleControllerServer = onrobot_rg_control.OnRobotRGSimpleControllerServer:main',
        ],
    },
)