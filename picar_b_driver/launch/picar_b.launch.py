import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_desc = get_package_share_directory('picar_b_description')
    urdf_file = os.path.join(pkg_desc, 'urdf', 'picarb.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
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
            package='camera_ros',
            executable='camera_node',
            name='picar_camera',
            output='screen',
            parameters=[{
                'width': 640,
                'height': 480,
                'format': 'RGB888',
            }],
            remappings=[
                ('~/image_raw', '/camera/image_raw'),
                ('~/camera_info', '/camera/camera_info'),
                ('~/image_raw/compressed', '/camera/image_raw/compressed'),
            ],
        ),
    ])
