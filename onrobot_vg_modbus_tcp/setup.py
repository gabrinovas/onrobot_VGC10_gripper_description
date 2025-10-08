from setuptools import setup
import os
from glob import glob

package_name = 'onrobot_vg_modbus_tcp'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pymodbus==2.5.3'],
    zip_safe=True,
    maintainer='Takuya Kiyokawa',
    maintainer_email='kiyokawa@hlab.sys.es.osaka-u.ac.jp',
    description='Modbus/TCP communication for OnRobot VG10 and VGC10 vacuum grippers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # No executable nodes in this package - it's a library
        ],
    },
)