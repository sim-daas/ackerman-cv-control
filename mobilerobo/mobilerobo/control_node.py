#!/usr/bin/env python3
"""
control_node.py — Visual-servoing controller for an Ackerman-steered robot.

Control Architecture
--------------------
Two independent PID loops driven by the perception node output:

  1. STEERING PID
     Error  : cx - (img_w / 2)   [pixels; +ve = target right of centre]
     Output : angular.z           [rad/s; cmd_vel]

  2. SPEED PID
     Error  : area - target_area  [px²; +ve = robot too far → drive forward]
     Output : linear.x            [m/s; cmd_vel]

Ackerman Kinematic Constraint
------------------------------
The robot CANNOT spin in place.  This node enforces:
  • If angular.z is non-zero, linear.x is at least `min_linear_speed`.
  • The robot stops (linear.x = 0) only when it is centred AND at the
    correct distance (area ≈ target_area).

State Machine
-------------
  SEARCHING  : No target visible. Robot rotates gently to scan.
               → angular.z = SCAN_TURN_SPEED, linear.x = min_linear_speed
  TRACKING   : Target visible. PID drives the robot toward the box.
  ARRIVED    : Area ≥ target_area and |cx_error| < threshold → stop.

Topics
------
Subscribes : /perception/target_info       (std_msgs/String)  — JSON
Publishes  : /ackermann_steering_controller/cmd_vel
               (geometry_msgs/TwistStamped)
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped


class PIDController:
    """Simple discrete PID with anti-windup clamp."""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float, output_max: float):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min = output_min
        self.out_max = output_max
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None

    def compute(self, error: float) -> float:
        now = time.monotonic()
        if self._prev_time is None:
            dt = 0.033          # assume ~30 Hz on first call
        else:
            dt = max(now - self._prev_time, 1e-6)
        self._prev_time = now

        self._integral   += error * dt
        derivative        = (error - self._prev_error) / dt
        self._prev_error  = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        # Anti-windup: clamp integral when output saturates
        if output > self.out_max:
            output = self.out_max
            self._integral -= error * dt
        elif output < self.out_min:
            output = self.out_min
            self._integral -= error * dt

        return output


class ControlNode(Node):

    # State constants
    SEARCHING = 'SEARCHING'
    TRACKING  = 'TRACKING'
    ARRIVED   = 'ARRIVED'

    # Scan speed when no target is visible (rad/s & m/s)
    SCAN_TURN_SPEED   = 0.25
    SCAN_FORWARD_SPEED = 0.08

    def __init__(self):
        super().__init__('control_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('steer_kp',  0.003)
        self.declare_parameter('steer_ki',  0.0)
        self.declare_parameter('steer_kd',  0.0005)

        self.declare_parameter('speed_kp',  0.000015)
        self.declare_parameter('speed_ki',  0.0)
        self.declare_parameter('speed_kd',  0.000002)

        self.declare_parameter('target_area',       18000)
        self.declare_parameter('max_linear_speed',  0.4)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('min_linear_speed',  0.05)

        # ── PID controllers ───────────────────────────────────────────────────
        # Built lazily after parameters are set; rebuilt on first message
        self._steer_pid: PIDController | None = None
        self._speed_pid: PIDController | None = None

        # ── State ─────────────────────────────────────────────────────────────
        self._state          = self.SEARCHING
        self._last_target_t  = 0.0   # epoch time of last received target
        self._lost_timeout   = 1.5   # seconds before reverting to SEARCHING

        # Arrival thresholds
        self._area_tolerance   = 1500   # px² — within this of target_area → arrived
        self._centre_tolerance = 40     # px  — within this of image centre → centred

        # ── Pub / Sub ─────────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            String, '/perception/target_info', self._target_callback, 10
        )
        self._pub = self.create_publisher(
            TwistStamped, '/ackermann_steering_controller/reference', 10
        )

        # Watchdog timer: if no target received, send stop/scan command
        self._watchdog = self.create_timer(0.1, self._watchdog_callback)

        self.get_logger().info('ControlNode started — Ackerman visual servoing ready.')

    # ── PID factory (reads current params) ────────────────────────────────────

    def _build_pids(self):
        max_lin = self.get_parameter('max_linear_speed').value
        max_ang = self.get_parameter('max_angular_speed').value

        self._steer_pid = PIDController(
            kp=self.get_parameter('steer_kp').value,
            ki=self.get_parameter('steer_ki').value,
            kd=self.get_parameter('steer_kd').value,
            output_min=-max_ang,
            output_max= max_ang,
        )
        self._speed_pid = PIDController(
            kp=self.get_parameter('speed_kp').value,
            ki=self.get_parameter('speed_ki').value,
            kd=self.get_parameter('speed_kd').value,
            output_min=0.0,         # never command reverse from speed PID
            output_max=max_lin,
        )
        self.get_logger().info('PID controllers (re)built from current parameters.')

    # ── Command helpers ────────────────────────────────────────────────────────

    def _publish_cmd(self, linear_x: float, angular_z: float):
        """Clamp, enforce Ackerman constraint, and publish TwistStamped."""
        max_lin = self.get_parameter('max_linear_speed').value
        max_ang = self.get_parameter('max_angular_speed').value
        min_lin = self.get_parameter('min_linear_speed').value

        linear_x  = float(max(-max_lin, min(max_lin, linear_x)))
        angular_z = float(max(-max_ang, min(max_ang, angular_z)))

        # ── Ackerman constraint: no spinning in place ──────────────────────
        if abs(angular_z) > 0.01 and abs(linear_x) < min_lin:
            linear_x = math.copysign(min_lin, linear_x) if linear_x != 0.0 else min_lin

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x  = linear_x
        msg.twist.angular.z = angular_z
        self._pub.publish(msg)

    def _stop(self):
        self._publish_cmd(0.0, 0.0)

    # ── Perception callback ────────────────────────────────────────────────────

    def _target_callback(self, msg: String):
        if self._steer_pid is None:
            self._build_pids()

        try:
            data = json.loads(msg.data)
            cx    = float(data['cx'])
            area  = float(data['area'])
            img_w = float(data['img_w'])
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f'Bad perception message: {e}')
            return

        self._last_target_t = time.monotonic()

        target_area   = self.get_parameter('target_area').value
        img_cx        = img_w / 2.0

        # ── Errors ────────────────────────────────────────────────────────────
        cx_error   = cx - img_cx              # +ve → box is right  → turn right
        area_error = target_area - area       # +ve → too far        → drive forward

        # ── Arrival check ────────────────────────────────────────────────────
        if (abs(area_error) < self._area_tolerance
                and abs(cx_error) < self._centre_tolerance):
            if self._state != self.ARRIVED:
                self.get_logger().info('ARRIVED at target box — stopping.')
            self._state = self.ARRIVED
            self._stop()
            self._steer_pid.reset()
            self._speed_pid.reset()
            return

        self._state = self.TRACKING

        # ── PID compute ───────────────────────────────────────────────────────
        # Negate steer output: positive cx_error → box is right → turn right
        # In ROS convention, positive angular.z → turn left, so negate.
        angular_z = -self._steer_pid.compute(cx_error)
        linear_x  =  self._speed_pid.compute(area_error)

        self._publish_cmd(linear_x, angular_z)

        self.get_logger().debug(
            f'[TRACKING] cx_err={cx_error:.1f}  area_err={area_error:.0f}  '
            f'lin={linear_x:.3f}  ang={angular_z:.3f}'
        )

    # ── Watchdog: handle lost target ──────────────────────────────────────────

    def _watchdog_callback(self):
        if self._state == self.ARRIVED:
            return

        elapsed = time.monotonic() - self._last_target_t
        if elapsed > self._lost_timeout:
            if self._state != self.SEARCHING:
                self.get_logger().info('Target lost — switching to SEARCHING (scan turn).')
                self._state = self.SEARCHING
                if self._steer_pid:
                    self._steer_pid.reset()
                if self._speed_pid:
                    self._speed_pid.reset()

            # Gentle forward+turn scan (Ackerman: must move to steer)
            self._publish_cmd(self.SCAN_FORWARD_SPEED, self.SCAN_TURN_SPEED)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
