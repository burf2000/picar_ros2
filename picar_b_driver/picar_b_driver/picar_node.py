#!/usr/bin/env python3
"""ROS2 driver node for Adeept PiCar-B.

Subscribes to cmd_vel (Twist) for differential drive control.
Publishes ultrasonic range data and line tracking sensor data.
Provides servo control for camera pan/tilt and wheel steering.
"""
import sys
import time

# Add Adeept server code to path so we can import GUImove directly
sys.path.insert(0, "/home/burf2000/adeept_picar-b/server")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, JointState
from std_msgs.msg import Int8MultiArray, Float64

import RPi.GPIO as GPIO

# ── Ultrasonic pins (BCM) ───────────────────────────────────────
ULTRA_TRIG = 11
ULTRA_ECHO = 8

# ── Line tracking pins (BCM) ────────────────────────────────────
LINE_RIGHT  = 19
LINE_MIDDLE = 16
LINE_LEFT   = 20


# ── Motor driver (uses Adeept GUImove directly) ─────────────────
class MotorDriver:
    def __init__(self):
        import GUImove
        self._move = GUImove
        self._move.setup()

    def drive(self, left_speed, right_speed):
        """Drive with left/right speeds from -100 to 100."""
        speed = max(abs(left_speed), abs(right_speed))
        if speed < 5:
            self.stop()
            return
        avg = (left_speed + right_speed) / 2.0
        if avg > 0:
            self._move.move(int(speed), "backward")  # "backward" = physically forward
        elif avg < 0:
            self._move.move(int(speed), "forward")   # "forward" = physically backward
        else:
            self.stop()

    def stop(self):
        self._move.motorStop()

    def cleanup(self):
        self._move.motorStop()
        self._move.destroy()


# ── Ultrasonic helper ────────────────────────────────────────────
class UltrasonicSensor:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ULTRA_TRIG, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(ULTRA_ECHO, GPIO.IN)

    def read_distance(self):
        """Returns distance in metres, or -1.0 on timeout."""
        GPIO.output(ULTRA_TRIG, GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(ULTRA_TRIG, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(ULTRA_TRIG, GPIO.LOW)
        timeout = time.time() + 0.04
        while not GPIO.input(ULTRA_ECHO):
            if time.time() > timeout:
                return -1.0
        t1 = time.time()
        timeout = t1 + 0.04
        while GPIO.input(ULTRA_ECHO):
            if time.time() > timeout:
                return -1.0
        t2 = time.time()
        return (t2 - t1) * 340.0 / 2.0


# ── Line tracker helper ─────────────────────────────────────────
class LineTracker:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for pin in [LINE_LEFT, LINE_MIDDLE, LINE_RIGHT]:
            GPIO.setup(pin, GPIO.IN)

    def read(self):
        return [
            GPIO.input(LINE_LEFT),
            GPIO.input(LINE_MIDDLE),
            GPIO.input(LINE_RIGHT),
        ]


# ── Servo helper (PCA9685) ──────────────────────────────────────
class ServoDriver:
    def __init__(self):
        try:
            import Adafruit_PCA9685
            self.pwm = Adafruit_PCA9685.PCA9685()
            self.pwm.set_pwm_freq(50)
            self.available = True
        except Exception:
            self.available = False

        # channel 0 = head tilt (inverted: higher=up)
        # channel 1 = head pan (lower=left)
        # channel 2 = wheel steering
        self.init_pos = {0: 300, 1: 300, 2: 300}
        self.pos = dict(self.init_pos)
        if self.available:
            for ch, val in self.init_pos.items():
                self.pwm.set_pwm(ch, 0, val)

    def set_angle(self, channel, pulse):
        """Set servo pulse (roughly 100-500 for 0-180 deg)."""
        pulse = max(100, min(500, int(pulse)))
        if self.available:
            self.pwm.set_pwm(channel, 0, pulse)
            self.pos[channel] = pulse

    def center_all(self):
        if self.available:
            for ch, val in self.init_pos.items():
                self.pwm.set_pwm(ch, 0, val)
                self.pos[ch] = val


# ── Main ROS2 Node ──────────────────────────────────────────────
class PicarBNode(Node):
    def __init__(self):
        super().__init__('picar_b_driver')

        # Parameters
        self.declare_parameter('max_speed', 80.0)
        self.declare_parameter('wheel_separation', 0.15)
        self.declare_parameter('ultrasonic_rate', 10.0)
        self.declare_parameter('line_track_rate', 20.0)
        self.declare_parameter('cmd_vel_timeout', 2.0)

        self.max_speed = self.get_parameter('max_speed').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        # Hardware
        self.motor = MotorDriver()
        self.ultrasonic = UltrasonicSensor()
        self.line_tracker = LineTracker()
        self.servo = ServoDriver()

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.servo_pan_sub = self.create_subscription(
            Float64, 'servo/pan', self.servo_pan_callback, 10)
        self.servo_tilt_sub = self.create_subscription(
            Float64, 'servo/tilt', self.servo_tilt_callback, 10)
        self.steering_sub = self.create_subscription(
            Float64, 'steering', self.steering_callback, 10)

        # Publishers
        self.range_pub = self.create_publisher(Range, 'ultrasonic', 10)
        self.line_pub = self.create_publisher(Int8MultiArray, 'line_track', 10)
        joint_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', joint_qos)

        # Timers
        ultra_period = 1.0 / self.get_parameter('ultrasonic_rate').value
        line_period = 1.0 / self.get_parameter('line_track_rate').value
        self.create_timer(ultra_period, self.publish_ultrasonic)
        self.create_timer(line_period, self.publish_line_track)
        self.create_timer(0.1, self.check_cmd_vel_timeout)
        self.create_timer(0.05, self.publish_joint_states)  # 20 Hz

        self.last_cmd_vel_time = self.get_clock().now()

        self.get_logger().info('PiCar-B driver node started')
        self.get_logger().info(f'  Servos available: {self.servo.available}')
        self.get_logger().info(f'  Max speed: {self.max_speed}')

    # ── cmd_vel → differential drive ────────────────────────────
    def cmd_vel_callback(self, msg: Twist):
        self.get_logger().info(f'cmd_vel: linear={msg.linear.x:.2f} angular={msg.angular.z:.2f}')
        self.last_cmd_vel_time = self.get_clock().now()
        linear = msg.linear.x   # m/s  (forward +)
        angular = msg.angular.z  # rad/s (left +)

        # Convert to left/right wheel speeds (-100..100)
        left  = linear - angular * self.wheel_sep / 2.0
        right = linear + angular * self.wheel_sep / 2.0

        # Normalise to -100..100
        max_val = max(abs(left), abs(right), 0.001)
        scale = min(1.0, 1.0 / max_val) * self.max_speed

        self.motor.drive(left * scale, right * scale)

    def check_cmd_vel_timeout(self):
        elapsed = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
        if elapsed > self.cmd_vel_timeout:
            self.motor.stop()

    # ── Servo callbacks ─────────────────────────────────────────
    def servo_pan_callback(self, msg: Float64):
        # Channel 1 = head pan (lower pulse = left)
        pulse = 300 - int(msg.data * 150)
        self.servo.set_angle(1, pulse)

    def servo_tilt_callback(self, msg: Float64):
        # Channel 0 = head tilt (inverted: higher pulse = up)
        pulse = 300 + int(msg.data * 100)
        self.servo.set_angle(0, pulse)

    def steering_callback(self, msg: Float64):
        # Channel 2 = wheel steering (higher pulse = left)
        pulse = 300 + int(msg.data * 100)
        self.get_logger().info(f'Steering: data={msg.data:.2f} pulse={pulse}')
        self.servo.set_angle(2, pulse)

    # ── Sensor publishers ───────────────────────────────────────
    def publish_ultrasonic(self):
        dist = self.ultrasonic.read_distance()
        if dist < 0:
            return
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # ~15 degrees
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = float(dist)
        self.range_pub.publish(msg)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['camera_pan_joint', 'camera_tilt_joint', 'steering_joint']
        # Invert the servo PWM mapping to recover radians
        pan_rad = (300 - self.servo.pos.get(1, 300)) / 150.0
        tilt_rad = (self.servo.pos.get(0, 300) - 300) / 100.0
        steer_rad = (300 - self.servo.pos.get(2, 300)) / 150.0
        msg.position = [pan_rad, tilt_rad, steer_rad]
        self.joint_pub.publish(msg)

    def publish_line_track(self):
        values = self.line_tracker.read()
        msg = Int8MultiArray()
        msg.data = values
        self.line_pub.publish(msg)

    # ── Cleanup ─────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info('Shutting down PiCar-B driver')
        self.motor.cleanup()
        self.servo.center_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PicarBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()


if __name__ == '__main__':
    main()
