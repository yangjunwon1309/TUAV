# launch/apriltag_land_vel.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_velocity_tracker', 
            executable='pitag_land_vel',
            name='pitag_land_vel',
            output='screen',
            parameters=[{
                'p_gain': 1.0,
                'max_vel': 0.5,
                'descent_vel': 0.1,
                'pos_thresh': 0.2,
                'timeout_sec': 2.0,
                'takeoff_height': 2.5,
                'takeoff_thresh': 0.2,
                'search_step': 0.2,
                'mission_csv': PathJoinSubstitution([
                    ThisLaunchFileDir(), '..', 'mission_csv', 'pitag_test.csv'
                ]),
            }]
        ),
        # static transform: base_link → cam/base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_camera',
            arguments=[
                '0.047064', '0.124641', '-0.217502',           # x y z translation
                '-0.7091122', '0.7049248', '0.0057976', '0.0143987',
                'base_link','camera_color_optical_frame2'
            ]
        ),
    ])