from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='picar_b_driver',
            executable='picar_node',
            name='picar_b_driver',
            output='screen',
            parameters=[{
                'max_speed': 80.0,
                'wheel_separation': 0.15,
                'ultrasonic_rate': 10.0,
                'line_track_rate': 20.0,
                'cmd_vel_timeout': 0.5,
            }],
        ),
        Node(
            package='picar_b_driver',
            executable='camera_node',
            name='picar_camera',
            output='screen',
            parameters=[{
                'width': 640,
                'height': 480,
                'fps': 15.0,
                'frame_id': 'camera_link',
            }],
        ),
    ])
