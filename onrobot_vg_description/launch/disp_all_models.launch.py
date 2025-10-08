from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='OnRobot VG Description Models Available:'),
        LogInfo(msg='  VG10: ros2 launch onrobot_vg_description disp_vg10_model.launch.py'),
        LogInfo(msg='  VGC10 (1 cup): ros2 launch onrobot_vg_description disp_vgc10_1cup_model.launch.py'),
        LogInfo(msg='  VGC10 (4 cups): ros2 launch onrobot_vg_description disp_vgc10_4cups_model.launch.py'),
        LogInfo(msg=''),
        LogInfo(msg='Usage examples:'),
        LogInfo(msg='  Display VG10 model: ros2 launch onrobot_vg_description disp_vg10_model.launch.py'),
        LogInfo(msg='  Display VGC10 1-cup: ros2 launch onrobot_vg_description disp_vgc10_1cup_model.launch.py'),
    ])