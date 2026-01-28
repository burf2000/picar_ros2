# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 Humble driver for the Adeept PiCar-B Mars Rover running on a Raspberry Pi (hostname: `picar`, IP: 192.168.20.110, Ubuntu 22.04 arm64).

Two ament_python packages:
- **picar_b_driver** — Hardware driver node (`picar_node`) and camera node (`camera_node`) that control motors, servos, ultrasonic sensor, line trackers, and Pi Camera via GPIO/I2C/libcamera
- **picar_b_description** — URDF robot model and visualization launch file

## Build & Test Commands

```bash
# Build all packages
colcon build --packages-select picar_b_driver picar_b_description

# Source after build
source install/setup.bash

# Run linter tests
colcon test --packages-select picar_b_driver
colcon test-result --verbose

# Launch driver node (on Pi only — requires GPIO hardware)
ros2 launch picar_b_driver picar_b.launch.py

# Launch URDF visualization (for RViz)
ros2 launch picar_b_description display.launch.py
```

## Deployment to Pi

```bash
scp -r picar_b_driver/ picar_b_description/ burf2000@picar.local:~/ros2_ws/src/
# Then SSH to Pi, cd ~/ros2_ws, colcon build, source, launch
```

## Architecture

### picar_node.py — the single driver node

Contains four hardware helper classes and the main ROS2 node:

- **MotorDriver** — differential drive via Adeept's `GUImove` library (GPIO 4,14,15,17,27,18). Note: GUImove "backward" = physically forward.
- **UltrasonicSensor** — HC-SR04 on GPIO 11 (trigger) / GPIO 8 (echo), returns distance in meters.
- **LineTracker** — 3 IR sensors on GPIO 20, 16, 19; returns binary array [left, middle, right].
- **ServoDriver** — PCA9685 I2C PWM controller. Channel 0=camera_tilt, 1=camera_pan, 2=wheel_steering (steering servo is burnt out).

**Subscriptions:** `/cmd_vel` (Twist), `/servo/pan` (Float64), `/servo/tilt` (Float64), `/steering` (Float64)

**Publishers:** `/ultrasonic` (Range at 10Hz), `/line_track` (Int8MultiArray at 20Hz), `/joint_states` (JointState at 20Hz — pan, tilt, steering positions)

**Parameters:** `max_speed` (80.0), `wheel_separation` (0.15m), `ultrasonic_rate` (10.0Hz), `line_track_rate` (20.0Hz), `cmd_vel_timeout` (0.5s auto-stop)

### URDF model (picarb.urdf)

Plain URDF (not xacro). Key joints: rear wheels (continuous/driven), front Ackermann steering (revolute ±0.6 rad), camera pan (±π/2) and tilt (-0.5 to 0.8 rad), ultrasonic sensor (fixed to camera tilt link).

### Servo PWM mapping

- Pan: `300 - msg.data * 150` (lower pulse = left)
- Tilt: `300 + msg.data * 100` (higher pulse = up, inverted)
- Steering: `300 + msg.data * 150`

### camera_node.py — Pi Camera driver

Uses `picamera2` (libcamera) and `cv_bridge` to publish:
- `/camera/image_raw` (Image) and `/camera/camera_info` (CameraInfo)
- Parameters: `width` (640), `height` (480), `fps` (15.0), `frame_id` ("camera_link")

### Hardware dependency

`picar_node.py` adds `~/adeept_picar-b/server` to `sys.path` to import the Adeept `GUImove` module. This code only runs on the Pi.
