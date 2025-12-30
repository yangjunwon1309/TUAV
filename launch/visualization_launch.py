from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # # Static transform from base_link to camera -- old (not correct)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=[
        #         '--x', '-0.006', '--y', '-0.208', '--z', '0.0526',
        #         '--yaw', '-1.6019346', '--pitch', '0.0219043', '--roll', '-3.096561',
        #         '--frame-id', 'base_link', '--child-frame-id', 'camera_color_optical_frame']
        # ),
        # # Static transform from base_link to camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_camera',
            arguments=[
                '0.210281', '0.000107', '-0.043040',           # x y z translation
                '-0.695617', '0.717976', '-0.008544', '0.023531',
                'base_link','camera_color_optical_frame'
            ]
        ),

        # Visualizer C++ node
        Node(
            package='waypoint_velocity_tracker',  # <-- Replace with actual package name
            executable='visualizer_node',  # <-- Now the compiled C++ binary name
            name='visualizer_node',
            output='screen'
        )
    ])
