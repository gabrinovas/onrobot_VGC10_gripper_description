from setuptools import setup
import os
from glob import glob

package_name = 'onrobot_vg_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takuya Kiyokawa',
    maintainer_email='kiyokawa@hlab.sys.es.osaka-u.ac.jp',
    description='Package to control OnRobot VG10 and VGC10 vacuum grippers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'OnRobotVGSimpleController = onrobot_vg_control.OnRobotVGSimpleController:main',
            'OnRobotVGStatusListener = onrobot_vg_control.OnRobotVGStatusListener:main',
            'OnRobotVGTcpNode = onrobot_vg_control.OnRobotVGTcpNode:main',
            'OnRobotVGSimpleControllerServer = onrobot_vg_control.OnRobotVGSimpleControllerServer:main',
        ],
    },
)