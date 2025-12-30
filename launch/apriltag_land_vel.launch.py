# launch/apriltag_land_vel.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_velocity_tracker', 
            executable='apriltag_land_vel',
            name='apriltag_land_vel',
            output='screen',
            parameters=[{
                'p_gain': 1.0,
                'max_vel': 0.5,
                'descent_vel': 0.1,
                'pos_thresh': 0.2,
                'timeout_sec': 2.0,
                'takeoff_height': 1.5,
                'takeoff_thresh': 0.2,
                'search_step': 0.3,
                'mission_csv': PathJoinSubstitution([
                    ThisLaunchFileDir(), '..', 'mission_csv', 'sim_yaw.csv'
                ]),
            }]
        ),
        # static transform: base_link → mono_cam/base_link
        #from base_link to camera
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=[
        #         '--x', '-0.006', '--y', '-0.208', '--z', '0.0526',
        #         '--yaw', '-1.6019346', '--pitch', '0.0219043', '--roll', '-3.096561',
        #         '--frame-id', 'base_link', '--child-frame-id', 'camera_color_optical_frame']
        # )
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_camera',
            arguments=[
                '0.210281', '0.000107', '-0.043040',           # x y z translation
                '-0.695617', '0.717976', '-0.008544', '0.023531',
                'base_link','camera_color_optical_frame2'
            ]
        )
    ])
