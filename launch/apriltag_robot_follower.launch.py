# launch/apriltag_land_vel.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_velocity_tracker', 
            executable='apriltag_robot_follower',
            name='apriltag_robot_follower',
            output='screen',
            parameters=[{
                'p_gain': 1.0,
                'max_vel': 0.5,
                'descent_vel': 0.1,
                'pos_thresh': 0.2,
                'timeout_sec': 1.0,
                'takeoff_height': 1.5,
                'takeoff_thresh': 0.2,
                'search_step': 0.3,
                'approach_thresh': 3.5,
            }]
        ),
        # static transform: base_link → mono_cam/base_link
        #from base_link to camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '-0.006', '--y', '-0.208', '--z', '0.0526',
                '--yaw', '-1.6019346', '--pitch', '0.0219043', '--roll', '-3.096561',
                '--frame-id', 'base_link', '--child-frame-id', 'camera_color_optical_frame']
        )
    ])
