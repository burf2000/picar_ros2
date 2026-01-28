"""Desktop launch file for PiCar-B visualisation and control.

Launches RViz with the robot model, camera feed, and ultrasonic display.
Also launches joystick teleop for driving the car.
Connects to the Pi's ROS2 topics over the network via DDS multicast.
The Pi must be running: ros2 launch picar_b_driver picar_b.launch.py
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('picar_b_description')
    rviz_config = os.path.join(pkg_share, 'rviz', 'picar.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
        # Joystick driver - reads USB gamepad
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }],
        ),
        # Teleop - converts joystick to cmd_vel
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[{
                'axis_linear.x': 1,      # Left stick Y
                'axis_angular.yaw': 0,   # Left stick X
                'scale_linear.x': 0.5,   # Max linear speed
                'scale_angular.yaw': 1.0,  # Max angular speed
                'enable_button': 4,      # L1 button - hold to drive
                'enable_turbo_button': 5,  # R1 button - hold for turbo
                'scale_linear_turbo.x': 1.0,
            }],
        ),
    ])
