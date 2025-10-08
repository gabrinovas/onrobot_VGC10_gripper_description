from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ip',
            default_value='192.168.1.1',
            description='IP address of the VG gripper'
        ),
        DeclareLaunchArgument(
            'port', 
            default_value='502',
            description='Port number'
        ),
        DeclareLaunchArgument(
            'changer_addr',
            default_value='65',
            description='Changer address'
        ),
        DeclareLaunchArgument(
            'dummy',
            default_value='false',
            description='Use dummy mode'
        ),
        
        Node(
            package='onrobot_vg_control',
            executable='OnRobotVGStatusListener',
            name='OnRobotVGStatusListener',
            output='screen'
        ),
        Node(
            package='onrobot_vg_control',
            executable='OnRobotVGTcpNode',
            name='OnRobotVGTcpNode',
            output='screen',
            parameters=[{
                'ip': LaunchConfiguration('ip'),
                'port': LaunchConfiguration('port'),
                'changer_addr': LaunchConfiguration('changer_addr'),
                'dummy': LaunchConfiguration('dummy')
            }]
        ),
        Node(
            package='onrobot_vg_control',
            executable='OnRobotVGSimpleControllerServer',
            name='OnRobotVGSimpleControllerServer',
            output='screen'
        )
    ])