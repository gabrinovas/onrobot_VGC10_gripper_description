from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'paused',
            default_value='false',
            description='Start Gazebo paused'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable Gazebo debug'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Enable Gazebo GUI'
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='125',
            description='Simulation rate'
        ),
        
        # Start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'paused': LaunchConfiguration('paused'),
                'debug': LaunchConfiguration('debug'),
                'gui': LaunchConfiguration('gui'),
            }.items()
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('onrobot_rg_description'),
                        'urdf/onrobot_rg6_model.xacro'
                    ])
                ])
            }]
        ),
        
        # Spawn URDF in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_urdf',
            arguments=[
                '-urdf',
                '-entity', 'onrobot_rg6',
                '-param', 'robot_description'
            ],
            output='screen'
        ),
        
        # Controller Spawner
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            output='screen',
            namespace='/onrobot_rg6',
            arguments=['joint_state_controller', 'joint_position_controller'],
            parameters=[{
                'use_sim_time': True
            }]
        )
    ])