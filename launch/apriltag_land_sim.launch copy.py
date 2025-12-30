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
                'takeoff_height': 2.5,
                'takeoff_thresh': 0.2,
                'search_step': 0.2,
                'mission_csv': PathJoinSubstitution([
                    ThisLaunchFileDir(), '..', 'mission_csv', 'sim_yaw.csv'
                ]),
            }]
        ),
        # static transform: base_link → mono_cam/base_link
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='camera_static_tf_pub',
        #     arguments=[
        #         '0', '0', '0.1',     # x y z (10cm above base_link)
        #         '0', '1.5708', '3.14159',  # RPY in radians (roll 0, pitch ~90°, yaw 0)
        #         'base_link',
        #         'x500_mono_cam_0/mono_cam/base_link/imager'
        #     ]
        # )
        #from base_link to camera - simulation
        # static transform: base_link → mono_cam/base_link
         # x y z (10cm above base_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '-0.1',
                '--yaw', '-1.5708', '--pitch', '0', '--roll', '3.14159',
                '--frame-id', 'base_link', '--child-frame-id', 'x500_mono_cam_0/mono_cam/base_link/imager']
        ),
    ])
