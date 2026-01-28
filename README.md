# PiCar-B ROS2 Driver

ROS2 Humble driver for the [Adeept PiCar-B](https://github.com/adeept/adeept_picar-b) Mars Rover robot running on a Raspberry Pi 3 (Ubuntu 22.04).

## Pi Details

- **Hostname:** `picar` (reachable as `picar.local` via mDNS)
- **IP:** 192.168.20.110
- **User:** `burf2000`
- **OS:** Ubuntu 22.04.5 LTS (arm64)
- **ROS2:** Humble (ros-humble-ros-base)

## ROS2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Subscribe | Differential drive: `linear.x` = forward/back, `angular.z` = turn |
| `/servo/pan` | `std_msgs/Float64` | Subscribe | Camera pan servo (-1.0 left to 1.0 right) |
| `/servo/tilt` | `std_msgs/Float64` | Subscribe | Camera tilt servo (-1.0 down to 1.0 up) |
| `/ultrasonic` | `sensor_msgs/Range` | Publish | Distance in metres (10 Hz) |
| `/line_track` | `std_msgs/Int8MultiArray` | Publish | [left, middle, right] IR sensors (0/1, 20 Hz) |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_speed` | 80.0 | Max PWM duty cycle (0-100) |
| `wheel_separation` | 0.15 | Distance between wheels in metres |
| `ultrasonic_rate` | 10.0 | Ultrasonic publish rate (Hz) |
| `line_track_rate` | 20.0 | Line tracking publish rate (Hz) |
| `cmd_vel_timeout` | 0.5 | Seconds without cmd_vel before motors auto-stop |

## Packages

| Package | Description |
|---------|-------------|
| `picar_b_driver` | Hardware driver node (motors, servos, sensors) |
| `picar_b_description` | URDF robot model and launch files |

## URDF

The robot model (`picar_b_description/urdf/picarb.urdf`) describes:

- **Chassis** (180x120x40mm box)
- **Rear wheels** (driven, continuous joints)
- **Front Ackermann steering** (revolute steering joint + front wheel axles)
- **Camera pan/tilt head** (pan = yaw on chassis, tilt = pitch)
- **Ultrasonic sensor** (mounted on camera tilt link)
- **Camera** (mounted on camera tilt link)

Joint mapping to PCA9685 servo channels:

| URDF Joint | Servo Channel | Notes |
|------------|---------------|-------|
| `camera_tilt_joint` | Ch 0 | Inverted: higher pulse = up |
| `camera_pan_joint` | Ch 1 | Lower pulse = left |
| `steering_joint` | Ch 2 | **Currently dead** (burnt out) |

Launch the robot model:

```bash
ros2 launch picar_b_description display.launch.py
```

## Running on the Pi

```bash
ssh burf2000@picar.local

# Launch the driver
ros2 launch picar_b_driver picar_b.launch.py

# Launch the robot model (separate terminal)
ros2 launch picar_b_description display.launch.py
```

## Controlling from Desktop

Make sure both machines use the same `ROS_DOMAIN_ID` (default 0) and are on the same network.

```bash
# Drive forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Turn left while moving
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}"

# Spin in place (turn right)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.5}}"

# Stop (Ctrl+C the pub command; motors auto-stop after 0.5s timeout)

# Pan camera left
ros2 topic pub --once /servo/pan std_msgs/msg/Float64 "{data: -1.0}"

# Read ultrasonic
ros2 topic echo /ultrasonic

# Read line sensors
ros2 topic echo /line_track
```

## Pi Setup (already done)

These steps have already been completed on the Pi:

1. **Swap:** 2GB swap file at `/swapfile`
2. **mDNS:** `avahi-daemon` installed for `picar.local` resolution
3. **ROS2 Humble:** Installed via apt (`ros-humble-ros-base`)
4. **Adeept code:** Cloned to `~/adeept_picar-b`
5. **Hardware libraries:** RPi.GPIO, Adafruit-PCA9685, rpi-ws281x, mpu6050-raspberrypi
6. **ROS2 workspace:** `~/ros2_ws/` with `picar_b_driver` and `picar_b_description` packages built
6b. **robot_state_publisher:** `ros-humble-robot-state-publisher` and `ros-humble-joint-state-publisher` installed
7. **Shell config:** `.bashrc` sources ROS2 and workspace automatically

## Deploying Changes

After editing the driver locally, deploy to the Pi:

```bash
scp -r picar_b_driver/ picar_b_description/ burf2000@picar.local:~/ros2_ws/src/

# Then on the Pi:
ssh burf2000@picar.local
cd ~/ros2_ws && colcon build --packages-select picar_b_driver picar_b_description
source install/setup.bash
```

## Hardware Pin Map (BCM)

| Function | Pin(s) |
|----------|--------|
| Motor A Enable | GPIO 4 |
| Motor A Dir | GPIO 14, 15 |
| Motor B Enable | GPIO 17 |
| Motor B Dir | GPIO 27, 18 |
| Ultrasonic Trig | GPIO 11 |
| Ultrasonic Echo | GPIO 8 |
| Line Track L/M/R | GPIO 20, 16, 19 |
| WS2812 LEDs | GPIO 12 |
| Servos | PCA9685 via I2C (ch 0=tilt, 1=pan, 2=steering) |
