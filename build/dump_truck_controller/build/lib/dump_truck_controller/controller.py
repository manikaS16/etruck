#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist
# from visualization_msgs.msg import Marker, MarkerArray

from dataclasses import dataclass, replace
from enum import Enum
from math import atan2, cos, sin, sqrt, pi
from typing import Callable, Deque, Dict, Iterable, Optional, Tuple
from collections import deque


def first_order_alpha(cutoff_hz: float, dt: float) -> float:
    return 0.0 if cutoff_hz <= 0.0 or dt <= 0.0 else 1.0 - pow(2.718281828, -2.0 * pi * cutoff_hz * dt)


@dataclass
class LowPass:
    cutoff_frequency: float
    last_filtered_value: float = 0.0
    initialized: bool = False

    def step(self, x: float, dt: float) -> float:
        if not self.initialized:
            self.last_filtered_value, self.initialized = x, True
            return x
        
        a = first_order_alpha(self.cutoff_frequency, dt)
        self.last_filtered_value = self.last_filtered_value + a * (x - self.last_filtered_value)
        return self.last_filtered_value
    
    def reset(self, value: float = 0.0) -> None:
        self.last_filtered_value, self.initialized = value, True


def clamp(value: float, min_value: float, max_value: float) -> float:
    return min(max(value, min_value), max_value)


@dataclass
class Params:
    pose_topic: str = "/vrpn_mocap/wheelloader/pose"
    cmd_topic: str = "/cmd_vel"

    target0_x: float = -1.8
    target0_y: float = 2.0
    target1_x: float = 0.0
    target1_y: float = 0.0

    position_tolerance: float = 0.01
    yaw_tolerance_deg: float = 1.0
    stop_hold_time: float = 1.0

    control_rate_freq: float = 10.0
    downsample_divisor: int = 2

    kp_pos: float = 0.8
    ki_pos: float = 0.05
    kd_pos: float = 0.0

    kp_yaw: float = 2.5
    ki_yaw: float = 0.05
    kd_yaw: float = 0.0

    k_ct: float = 1.0

    linear_velocity_min: float = 0.5
    linear_velocity_max: float = 1.0
    angular_velocity_min: float = -1.0
    angular_velocity_max: float = 1.0

    max_linear_acceleration: float = 1.0
    max_yaw_acceleration: float = 2.0

    cutoff_frequency_pos_d: float = 1.0
    cutoff_frequency_yaw_d: float = 1.0

    linear_velocity_during_turn: float = 0.2
    angular_velocity_during_turn: float = 0.8
    uturn_yaw_target_deg: float = 180.0
    uturn_timeout_sec: float = 4.0

    pose_timeout: float = 0.15
    resume_grace: float = 0.10


class State(str, Enum):
    HOLD_NO_POSE = "HOLD_NO_POSE"
    DRIVE_TO_P1 = "DRIVE_TO_PI"
    STOP_AT_P1 = "STOP_AT_PI"
    UTURN_AT_P1 = "UTURN_AT_PI"
    DRIVE_TO_P0 = "DRIVE_TO_P0"
    STOP_AT_P0 = "STOP_AT_P0"
    UTURN_AT_P0 = "UTURN_AT_P0"


@dataclass
class RateLimiter:
    max_rate: float
    previous_value: float = 0.0
    initialized: bool = False

    def step(self, target: float, dt: float) -> float:
        if not self.initialized:
            self.previous_value, self.initialized = target, True
            return target
        
        max_delta = self.max_rate * max(dt, 0.0)
        delta = clamp(target - self.previous_value, -max_delta, max_delta)
        self.previous_value += delta
        return self.previous_value
    
    def reset(self, value: float = 0.0) -> None:
        self.previous_value, self.initialized = value, True


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    integrator_lower_bound: float
    integral_upper_bound: float
    derivative_filter: LowPass
    accumulated_integral_value: float = 0.0
    previous_error: Optional[float] = None

    def reset(self) -> None:
        self.accumulated_integral_value = 0.0
        self.previous_error = None
        self.derivative_filter.reset(0.0)

    def step(self, error: float, dt: float) -> Tuple[float, Dict[str, float]]:
        p = self.kp * error
        
        if dt <= 0.0:
            d_raw, d = 0.0, 0.0
        else:
            d_raw = 0.0 if self.previous_error is None else (error - self.previous_error) / dt
            d = self.kd * self.derivative_filter.step(d_raw, dt)

        self.previous_error = error
        self.accumulated_integral_value = clamp(self.accumulated_integral_value + self.ki * error * max(dt, 0.0), self.integrator_lower_bound, self.integral_upper_bound)
        u = p + self.accumulated_integral_value + d
        return u, {"p": p, "i": self.accumulated_integral_value, "d_raw": d_raw, "d": d}
    
    def backoff_integrator(self, saturated: bool) -> None:
        if saturated:
            self.accumulated_integral_value = clamp(self.accumulated_integral_value, self.integrator_lower_bound, self.integral_upper_bound)



def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    s = 2.0 * (qw * qz + qx * qy)
    c = 1.0 - 2.0 * (qy * qy + qz * qz)
    return atan2(s, c)


def goal_for_state(state: State, p: Params) -> Tuple[float, float]:
    return (p.target1_x, p.target1_y) if state in (State.DRIVE_TO_P1, State.STOP_AT_P1, State.UTURN_AT_P1) else (p.target0_x, p.target0_y)


def vector_norm(x: float, y: float) -> float:
    return sqrt(x * x + y * y)


def wrap_to_pi(angle: float) -> float:
    a = (angle + pi) % (2.0 * pi)
    return a - pi


def compute_errors(px: float, py: float, yaw: float, gx: float, gy: float) -> Tuple[float, float, float, float]:
    ex_w, ey_w = gx - px, gy - py
    dist = vector_norm(ex_w, ey_w)
    heading = atan2(ey_w, ex_w)
    e_yaw_goal = wrap_to_pi(heading - yaw)
    c, s = cos(-yaw), sin(-yaw)
    e_long = c * ex_w - s * ey_w
    e_cross = s * ex_w + c * ey_w
    return e_long, e_cross, e_yaw_goal, dist


def pid_outputs(e_long: float, e_yaw: float, pos_pid: PID, yaw_pid: PID, p: Params, dt: float) -> Tuple[float, float, Dict[str, float]]:
    yaw_cmd_raw, yaw_dbg = yaw_pid.step(e_yaw, dt)
    v_cmd_raw, pos_dbg = pos_pid.step(e_long, dt)
    v_cmd = clamp(v_cmd_raw, p.linear_velocity_min, p.linear_velocity_max)
    w_cmd = clamp(yaw_cmd_raw, p.angular_velocity_min, p.angular_velocity_max)
    yaw_pid.backoff_integrator(yaw_cmd_raw != w_cmd)
    pos_pid.backoff_integrator(v_cmd_raw != v_cmd)
    dbg = {"pos": pos_dbg, "yaw": yaw_dbg}
    return v_cmd, w_cmd, dbg


def apply_slew(v: float, w: float, v_limiter: RateLimiter, w_limiter: RateLimiter, dt: float) -> Tuple[float, float]:
    return v_limiter.step(v, dt), w_limiter.step(w, dt)


def within_tolerance(e_long: float, e_cross: float, e_yaw: float, p: Params) -> bool:
    pos_ok = vector_norm(e_long, e_cross) <= p.position_tolerance
    yaw_ok = abs(e_yaw) <= (p.yaw_tolerance_deg * pi / 180.0)
    return pos_ok and yaw_ok


class Controller(Node):
    def __init__(self) -> None:
        super().__init__("dump_truck_controller")
        self.p = self._declare_params()

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST)

        self.pose_subscriber = self.create_subscription(PoseStamped, self.p.pose_topic, self._on_pose_callback, qos)
        self.pose_counter: int = 0
        self.pose_queue: Deque[PoseStamped] = deque(maxlen=1)
        self.last_pose_time: Optional[float] = None

        self.cmd_publisher = self.create_publisher(Twist, self.p.cmd_topic, 10)
        self.control_dt = 1.0 / self.p.control_rate_freq
        self.timer = self.create_timer(self.control_dt, self._on_timer_callback)
        self.state: State = State.DRIVE_TO_P1
        self.state_enter_time: float = 0.0
        self.last_pose_time: Optional[float] = None
        self.v_limiter = RateLimiter(self.p.max_linear_acceleration)
        self.w_limiter = RateLimiter(self.p.max_yaw_acceleration)
        self.pos_PID = PID(self.p.kp_pos, self.p.ki_pos, self.p.kd_pos, -0.5, 0.5, LowPass(self.p.cutoff_frequency_pos_d))
        self.yaw_PID = PID(self.p.kp_yaw, self.p.ki_yaw, self.p.kd_yaw, -0.5, 0.5, LowPass(self.p.cutoff_frequency_yaw_d))
        self.uturn_target_yaw: Optional[float] = None
        self.previous_yaw: Optional[float] = None

    def _declare_params(self) -> Params:
        get = self.declare_parameter
        p = Params(
            pose_topic=get("pose_topic", Params.pose_topic).value,
            cmd_topic=get("cmd_topic", Params.cmd_topic).value,
            target0_x=get("target0_x", Params.target0_x).value,
            target0_y=get("target0_y", Params.target0_y).value,
            target1_x=get("target1_x", Params.target1_x).value,
            target1_y=get("target1_y", Params.target1_y).value,
            position_tolerance=get("position_tolerance", Params.position_tolerance).value,
            yaw_tolerance_deg=get("yaw_tolerance_deg", Params.yaw_tolerance_deg).value,
            stop_hold_time=get("stop_hold_time", Params.stop_hold_time).value,
            control_rate_freq=get("control_rate_freq", Params.control_rate_freq).value,
            downsample_divisor=get("downsample_divisor", Params.downsample_divisor).value,
            kp_pos=get("kp_pos", Params.kp_pos).value,
            ki_pos=get("ki_pos", Params.ki_pos).value,
            kd_pos=get("kd_pos", Params.kd_pos).value,
            kp_yaw=get("kp_yaw", Params.kp_yaw).value,
            ki_yaw=get("ki_yaw", Params.ki_yaw).value,
            kd_yaw=get("kd_yaw", Params.kd_yaw).value,
            k_ct=get("k_ct", Params.k_ct).value,
            linear_velocity_min=get("linear_velocity_min", Params.linear_velocity_min).value,
            linear_velocity_max=get("linear_velocity_max", Params.linear_velocity_max).value,
            angular_velocity_min=get("angular_velocity_min", Params.angular_velocity_min).value,
            angular_velocity_max=get("angular_velocity_max", Params.angular_velocity_max).value,
            max_linear_acceleration=get("linear_acceleration_min", Params.max_linear_acceleration).value,
            max_yaw_acceleration=get("max_yaw_acceleration", Params.max_yaw_acceleration).value,
            cutoff_frequency_pos_d=get("cutoff_frequency_pos_d", Params.cutoff_frequency_pos_d).value,
            cutoff_frequency_yaw_d=get("cutoff_frequency_yaw_d", Params.cutoff_frequency_yaw_d).value,
            linear_velocity_during_turn=get("linear_velocity_during_turn", Params.linear_velocity_during_turn).value,
            angular_velocity_during_turn=get("angular_velocity_during_turn", Params.angular_velocity_during_turn).value,
            uturn_yaw_target_deg=get("uturn_yaw_target_deg", Params.uturn_yaw_target_deg).value,
            uturn_timeout_sec=get("uturn_timeout_sec", Params.uturn_timeout_sec).value,
            pose_timeout=get("pose_timeout", Params.pose_timeout).value,
            resume_grace=get("resume_grace", Params.resume_grace).value
        )
        return p
    
    def _stamp_sec(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec)
    
    def _on_pose_callback(self, msg: PoseStamped) -> None:
        self.pose_counter += 1
        
        if self.pose_counter % max(1, self.p.downsample_divisor) != 0:
            return
        
        self.pose_queue.append(msg)
        self.last_pose_time = self._stamp_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else self.get_clock().now().seconds_nanoseconds()[0] + 0.0

    def _enter(self, new_state: State) -> None:
        self.state = new_state
        self.state_enter_time = self.get_clock().now().seconds_nanoseconds()[0] + 0.0
        self.v_limiter.reset(0.0)
        self.w_limiter.reset(0.0)
        self.pos_PID.reset()
        self.yaw_PID.reset()
        self.uturn_target_yaw: Optional[float] = None
        self.get_logger().info(f"State -> {self.state.value}")

    def _publish_cmd(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_publisher.publish(msg)
    
    def _on_timer_callback(self) -> None:
        now = self.get_clock().now().seconds_nanoseconds()[0] + 0.0
        pose_age = float("inf") if self.last_pose_time is None else (now - self.last_pose_time)
        no_pose = pose_age > self.p.pose_timeout

        if no_pose and self.state is not State.HOLD_NO_POSE:
            self._enter(State.HOLD_NO_POSE)

        if self.state is State.HOLD_NO_POSE:
            if pose_age <= self.p.resume_grace:
                self._enter(State.DRIVE_TO_P1)

            self._publish_cmd(0.0, 0.0)
            return
        
        pose = self.pose_queue[-1] if self.pose_queue else None

        if not pose:
            self._publish_cmd(0.0, 0.0)
            return
        
        px = pose.pose.position.x
        py = pose.pose.position.y
        q = pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.previous_yaw = yaw if self.previous_yaw is None else self.previous_yaw

        gx, gy = goal_for_state(self.state, self.p)
        e_long, e_cross, e_yaw_goal, dist = compute_errors(px, py, yaw, gx, gy)
        e_yaw = wrap_to_pi(e_yaw_goal + self.p.k_ct * e_cross)

        dt = self.control_dt

        if self.state in (State.DRIVE_TO_P1, State.DRIVE_TO_P0):
            v_cmd, w_cmd, _ = pid_outputs(e_long, e_yaw, self.pos_PID, self.yaw_PID, self.p, dt)
            v_cmd, w_cmd = apply_slew(v_cmd, w_cmd, self.v_limiter, self.w_limiter, dt)
            reached = within_tolerance(e_long, e_cross, e_yaw_goal, self.p)
            self._publish_cmd(v_cmd, w_cmd)
            if reached:
                nxt = State.STOP_AT_P1 if self.state is State.DRIVE_TO_P1 else State.STOP_AT_P0
                self._enter(nxt)
        elif self.state in (State.STOP_AT_P1, State.STOP_AT_P0):
            self._publish_cmd(0.0, 0.0)
            if now - self.state_enter_time >= self.p.stop_hold_time:
                nxt = State.UTURN_AT_P1 if self.state is State.STOP_AT_P1 else State.UTURN_AT_P0
                self._enter(nxt)
        elif self.state in (State.UTURN_AT_P1, State.UTURN_AT_P0):
            if self.uturn_target_yaw is None:
                self.uturn_target_yaw = wrap_to_pi(yaw + self.p.uturn_yaw_target_deg * pi / 180.0)
            w_sign = 1.0 if wrap_to_pi(self.uturn_target_yaw - yaw) >= 0.0 else -1.0
            v_cmd = clamp(self.p.linear_velocity_during_turn, self.p.linear_velocity_min, self.p.linear_velocity_max)
            w_cmd = clamp(w_sign * abs(self.p.angular_velocity_during_turn), self.p.angular_velocity_min, self.p.angular_velocity_max)
            v_cmd, w_cmd = apply_slew(v_cmd, w_cmd, self.v_limiter, self.w_limiter, dt)
            self._publish_cmd(v_cmd, w_cmd)
            yaw_error = wrap_to_pi(self.uturn_target_yaw - yaw)
            timed_out = (now - self.state_enter_time) >= self.uturn_timerout_sec
            if abs(yaw_error) <= (self.p.yaw_tolerance_deg * pi / 180.0) or timed_out:
                nxt = State.DRIVE_TO_P0 if self.state is State.UTURN_AT_P1 else State.DRIVE_TO_P1
                self._enter(nxt)


def main(args: Optional[Iterable[str]] = None) -> None:
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
