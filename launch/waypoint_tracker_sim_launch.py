from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_velocity_tracker',
            executable='waypoint_velocity_tracker_node',
            name='waypoint_velocity_tracker',
            output='screen',
            parameters=[{
                'mission_csv': PathJoinSubstitution([
                    ThisLaunchFileDir(), '..', 'mission_csv', 'sim.csv'
                ])
            }]
        )
    ])
