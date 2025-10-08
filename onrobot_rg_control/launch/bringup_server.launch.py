from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ip',
            default_value='192.168.1.1',
            description='IP address of the gripper'
        ),
        DeclareLaunchArgument(
            'port', 
            default_value='502',
            description='Port number'
        ),
        DeclareLaunchArgument(
            'gripper',
            default_value='rg6',
            description='Gripper type (rg2 or rg6)'
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
            package='onrobot_rg_control',
            executable='OnRobotRGStatusListener',
            name='OnRobotRGStatusListener',
            output='screen'
        ),
        Node(
            package='onrobot_rg_control',
            executable='OnRobotRGTcpNode',
            name='OnRobotRGTcpNode',
            output='screen',
            parameters=[{
                'ip': LaunchConfiguration('ip'),
                'port': LaunchConfiguration('port'),
                'gripper': LaunchConfiguration('gripper'),
                'changer_addr': LaunchConfiguration('changer_addr'),
                'dummy': LaunchConfiguration('dummy')
            }]
        ),
        Node(
            package='onrobot_rg_control',
            executable='OnRobotRGSimpleControllerServer',
            name='OnRobotRGSimpleControllerServer',
            output='screen',
            parameters=[{
                'gripper': LaunchConfiguration('gripper')
            }]
        )
    ])