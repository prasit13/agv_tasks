from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare a launch argument to set 'v' and 'd' parameters for the talker node
        DeclareLaunchArgument('v', default_value='2.0', description='Speed parameter for the talker node'),
        DeclareLaunchArgument('d', default_value='1.0', description='Steering angle parameter for the talker node'),

        # Launch the talker node with parameters
        Node(
            package='lab1_pkg',
            executable='talker',
            name='talker',
            namespace='lab1_pkg',
            output='screen',
            parameters=[
                {'v': LaunchConfiguration('v')},
                {'d': LaunchConfiguration('d')}
            ],
        ),

        # Launch the relay node
        Node(
            package='lab1_pkg',
            executable='relay',
            name='relay',
            namespace='lab1_pkg',
            output='screen',
        ),

        # Log a message indicating the launch is complete
        LogInfo(
            condition=LaunchConfiguration('v').equals('2.0') & LaunchConfiguration('d').equals('1.0'),
            value="Nodes 'talker' and 'relay' are launched with parameters: v=2.0 and d=1.0."
        ),
    ])
