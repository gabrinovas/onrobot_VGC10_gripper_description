from setuptools import setup
import os
from glob import glob

package_name = 'onrobot_grippers_description'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all directories recursively
        (os.path.join('share', package_name), 
         glob('launch/*.launch.py') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'urdf', 'rg2'), 
         glob('urdf/rg2/*.xacro')),
        (os.path.join('share', package_name, 'urdf', 'rg6'), 
         glob('urdf/rg6/*.xacro')),
        (os.path.join('share', package_name, 'urdf', 'vg10'), 
         glob('urdf/vg10/*.xacro')),
        (os.path.join('share', package_name, 'urdf', 'vgc10'), 
         glob('urdf/vgc10/*.xacro')),
        # Recursive mesh installation
        (os.path.join('share', package_name, 'meshes'), 
         glob('meshes/**/*', recursive=True)),
        # Recursive image installation  
        (os.path.join('share', package_name, 'images'), 
         glob('images/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takuya Kiyokawa',
    maintainer_email='taku8926@gmail.com',
    description='OnRobot grippers description package (RG2, RG6, VG10, VGC10)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)