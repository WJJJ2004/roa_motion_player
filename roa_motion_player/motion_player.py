#!/usr/bin/env python3
import math
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
import yaml

from roa_interfaces.msg import MotorCommand, MotorCommandArray
from .gamepad_reader import Gamepad
from .mapping import (
    ARM_JOINT_TO_ID, CONFIGURED_MOTOR_IDS, LEFT_ARM_JOINT_TO_ID,
    MOTOR_GAINS, POSITION_LIMITS, RIGHT_ARM_JOINT_TO_ID,
)


class MotionPlayer(Node):
    WAIT_HW = "WAIT_HW"
    STARTUP_DELAY = "STARTUP_DELAY"
    IDLE = "IDLE"
    INTERPOLATE_TO_MOTION = "INTERPOLATE_TO_MOTION"
    MOTION_PLAY = "MOTION_PLAY"
    INTERPOLATE_TO_IDLE = "INTERPOLATE_TO_IDLE"

    CYAN = "\033[1;36m"
    GREEN = "\033[1;32m"
    YELLOW = "\033[1;33m"
    RESET = "\033[0m"

    def __init__(self):
        super().__init__("motion_player")
        self.command_topic = str(self.declare_parameter(
            "command_topic", "/hardware_interface/command").value)
        self.wait_for_walk_initialized = bool(self.declare_parameter(
            "wait_for_walk_initialized", False).value)
        self.walk_initialized_topic = str(self.declare_parameter(
            "walk_initialized_topic", "/walk_initialized").value)
        self.startup_delay = float(self.declare_parameter("startup_delay", 5.0).value)
        self.interpolation_time = float(self.declare_parameter(
            "interpolation_time", 1.0).value)
        self.publish_rate = float(self.declare_parameter("publish_rate_hz", 300.0).value)
        self.max_command_velocity = float(self.declare_parameter(
            "max_command_velocity_rad_s", 15.0).value)
        self.command_step_tolerance = float(self.declare_parameter(
            "command_step_tolerance_rad", 0.002).value)
        self.gamepad_enabled = bool(self.declare_parameter("gamepad_enabled", True).value)
        idle_file = str(self.declare_parameter(
            "idle_pose_file", "motions/idle_pose.yaml").value)
        catalog_file = str(self.declare_parameter(
            "motion_catalog_file", "config/motion_catalog.yaml").value)
        gain_file = str(self.declare_parameter(
            "gain_profiles_file", "config/gain_profiles.yaml").value)

        if self.publish_rate <= 0.0 or self.interpolation_time <= 0.0:
            raise ValueError("publish_rate_hz and interpolation_time must be positive")
        if self.max_command_velocity <= 0.0 or self.command_step_tolerance < 0.0:
            raise ValueError(
                "max_command_velocity_rad_s must be positive and "
                "command_step_tolerance_rad must be non-negative")
        if self.startup_delay < 0.0:
            raise ValueError("startup_delay must be non-negative")

        self.share = Path(get_package_share_directory("roa_motion_player"))
        self.idle_motion = self._load_motion(self._resolve(idle_file))
        self.idle_pose = self._pose_from_sample(self.idle_motion["samples"][0])
        self.default_gains = dict(MOTOR_GAINS)
        self.gain_profiles = self._load_gain_profiles(self._resolve(gain_file))
        self.motions, self.button_map = self._load_catalog(self._resolve(catalog_file))

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(
            MotorCommandArray, self.command_topic, command_qos)
        self.walk_initialized_subscription = self.create_subscription(
            Bool, self.walk_initialized_topic, self._on_walk_initialized, command_qos)

        self.gamepad = None
        if self.gamepad_enabled:
            device_path = str(self.declare_parameter("device_path", "").value)
            vendor_id = int(self.declare_parameter("vendor_id", 0x046D).value)
            product_id = int(self.declare_parameter("product_id", 0xC219).value)
            name_hint = str(self.declare_parameter(
                "prefer_name_contains", "Logitech").value)
            self.gamepad = Gamepad(
                vendor_id=vendor_id,
                product_id=product_id,
                vel_scale_x=1.0,
                vel_scale_y=1.0,
                vel_scale_rot=1.0,
                device_path=device_path or None,
                prefer_name_contains=name_hint,
            )

        self.walk_initialized = False
        self.gate_paused_at = None
        self.state = self.WAIT_HW if self.wait_for_walk_initialized else self.STARTUP_DELAY
        self.state_started = time.monotonic()
        self.current_pose = dict(self.idle_pose)
        self.current_gains = dict(self.default_gains)
        self.previous_command = None
        self.previous_publish_time = None
        self.active_motion = None
        self.transition_start_pose = dict(self.idle_pose)
        self.transition_target_pose = dict(self.idle_pose)
        self.transition_start_gains = dict(self.default_gains)
        self.transition_target_gains = dict(self.default_gains)
        self.timer = self.create_timer(1.0 / self.publish_rate, self._tick)

        mapping = ", ".join(
            f"{code}->{name}" for code, name in sorted(self.button_map.items()))
        self.get_logger().info(
            f"motion player ready; state={self.state}; interpolation="
            f"{self.interpolation_time:.3f}s; buttons: {mapping}")

    def _resolve(self, value):
        path = Path(value)
        return path if path.is_absolute() else self.share / path

    def _load_motion(self, path):
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        joints = list(data.get("joints", []))
        if not joints or len(set(joints)) != len(joints):
            raise ValueError(f"{path}: joints must be non-empty and unique")
        unknown = [joint for joint in joints if joint not in ARM_JOINT_TO_ID]
        if unknown:
            raise ValueError(f"{path}: unknown arm joints: {unknown}")
        rows = data.get("positions", [])
        if not rows or any(len(row) != len(joints) for row in rows):
            raise ValueError(f"{path}: position row width must match joints")
        period = float(data["sample_period"])
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError(f"{path}: invalid sample_period")
        samples = []
        for row in rows:
            sample = {}
            for joint, raw_value in zip(joints, row):
                motor_id = ARM_JOINT_TO_ID[joint]
                value = float(raw_value)
                lower, upper = POSITION_LIMITS[motor_id]
                if not math.isfinite(value) or not lower <= value <= upper:
                    raise ValueError(f"{path}: invalid position {value} for {joint}")
                sample[motor_id] = value
            samples.append(sample)
        return {
            "name": str(data["name"]), "path": path,
            "period": period, "samples": samples,
        }

    def _load_gain_profiles(self, path):
        with path.open("r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream).get("profiles", {})
        profiles = {}
        for name, settings in raw.items():
            gains = dict(self.default_gains)
            arm_scale = float(settings.get("arm_scale", 1.0))
            right_scale = float(settings.get("right_arm_scale", arm_scale))
            left_scale = float(settings.get("left_arm_scale", arm_scale))
            for motor_id in RIGHT_ARM_JOINT_TO_ID.values():
                kp, kd = self.default_gains[motor_id]
                gains[motor_id] = (kp * right_scale, kd * right_scale)
            for motor_id in LEFT_ARM_JOINT_TO_ID.values():
                kp, kd = self.default_gains[motor_id]
                gains[motor_id] = (kp * left_scale, kd * left_scale)
            profiles[str(name)] = gains
        if "default" not in profiles:
            raise ValueError(f"{path}: a default gain profile is required")
        return profiles

    def _load_catalog(self, path):
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        motions = {}
        for name, item in data.get("motions", {}).items():
            motion = self._load_motion(self._resolve(item["file"]))
            profile_name = str(item.get("gain_profile", "default"))
            if profile_name not in self.gain_profiles:
                raise ValueError(f"{path}: unknown gain profile {profile_name}")
            motion["gain_profile"] = profile_name
            motion["gains"] = self.gain_profiles[profile_name]
            motions[str(name)] = motion
        button_map = {int(code): str(name) for code, name in data.get("buttons", {}).items()}
        missing = [name for name in button_map.values() if name not in motions]
        if missing:
            raise ValueError(f"{path}: buttons reference unknown motions: {missing}")
        return motions, button_map

    @staticmethod
    def _zero_pose():
        return {motor_id: 0.0 for motor_id in CONFIGURED_MOTOR_IDS}

    def _pose_from_sample(self, sample):
        pose = self._zero_pose()
        pose.update(sample)
        return pose

    def _motion_pose(self, motion, sample_index):
        pose = dict(self.idle_pose)
        pose.update(motion["samples"][sample_index])
        return pose

    def _interpolated_motion_pose(self, motion, elapsed):
        """Linearly interpolate recorded samples at the current motion time."""
        last_index = len(motion["samples"]) - 1
        sample_position = max(0.0, elapsed / motion["period"])
        lower_index = min(last_index, int(math.floor(sample_position)))
        upper_index = min(last_index, lower_index + 1)
        alpha = min(1.0, max(0.0, sample_position - lower_index))

        lower = motion["samples"][lower_index]
        upper = motion["samples"][upper_index]
        arm_sample = {
            motor_id: lower[motor_id] + alpha * (upper[motor_id] - lower[motor_id])
            for motor_id in lower
        }
        return self._motion_pose_from_values(arm_sample)

    def _motion_pose_from_values(self, arm_values):
        pose = dict(self.idle_pose)
        pose.update(arm_values)
        return pose

    def _on_walk_initialized(self, message):
        initialized = bool(message.data)
        now = time.monotonic()
        if initialized and not self.walk_initialized:
            self.get_logger().info("/walk_initialized=true")
            if self.state == self.WAIT_HW:
                self._enter(self.STARTUP_DELAY)
            elif self.gate_paused_at is not None:
                self.state_started += now - self.gate_paused_at
            self.gate_paused_at = None
        elif not initialized and self.walk_initialized:
            self.get_logger().warning("/walk_initialized=false; FSM paused")
            self.gate_paused_at = now
        self.walk_initialized = initialized

    def _enter(self, state):
        self.state = state
        self.state_started = time.monotonic()

    def _start_interpolation(self, target_pose, target_gains, state):
        self.transition_start_pose = dict(self.current_pose)
        self.transition_target_pose = dict(target_pose)
        self.transition_start_gains = dict(self.current_gains)
        self.transition_target_gains = dict(target_gains)
        self._enter(state)

    def _interpolate(self, elapsed):
        alpha = min(1.0, elapsed / self.interpolation_time)
        pose = {
            motor_id: self.transition_start_pose[motor_id] + alpha * (
                self.transition_target_pose[motor_id] - self.transition_start_pose[motor_id])
            for motor_id in CONFIGURED_MOTOR_IDS
        }
        gains = {}
        for motor_id in CONFIGURED_MOTOR_IDS:
            start_kp, start_kd = self.transition_start_gains[motor_id]
            target_kp, target_kd = self.transition_target_gains[motor_id]
            gains[motor_id] = (
                start_kp + alpha * (target_kp - start_kp),
                start_kd + alpha * (target_kd - start_kd),
            )
        return pose, gains, alpha

    def _request_motion(self, name):
        if self.state != self.IDLE:
            return
        motion = self.motions.get(name)
        if motion is None:
            self.get_logger().warning(f"unknown motion request: {name}")
            return
        self.active_motion = motion
        target_pose = self._motion_pose(motion, 0)
        self._start_interpolation(
            target_pose, motion["gains"], self.INTERPOLATE_TO_MOTION)
        self.get_logger().info(
            f"{self.YELLOW}REQUEST {name} ({motion['path'].name}){self.RESET}")

    def _poll_gamepad(self):
        if self.gamepad is None:
            return
        buttons = self.gamepad.consume_pressed_buttons()
        if self.state != self.IDLE:
            if buttons:
                self.get_logger().info("button input ignored outside IDLE")
            return
        for code in buttons:
            name = self.button_map.get(code)
            if name is not None:
                self._request_motion(name)
                break

    def _publish(self):
        now = time.monotonic()
        if self.previous_command is not None:
            step = max(
                abs(self.current_pose[i] - self.previous_command[i])
                for i in CONFIGURED_MOTOR_IDS)
            dt = max(0.0, now - self.previous_publish_time)
            allowed_step = (
                self.max_command_velocity * dt + self.command_step_tolerance)
            if step > allowed_step:
                raise RuntimeError(
                    f"command discontinuity {step:.4f} rad exceeds "
                    f"velocity-based limit {allowed_step:.4f} rad "
                    f"(dt={dt:.6f}s, max_velocity="
                    f"{self.max_command_velocity:.3f}rad/s)")
        message = MotorCommandArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "roa_motion_player"
        for motor_id in CONFIGURED_MOTOR_IDS:
            lower, upper = POSITION_LIMITS[motor_id]
            kp, kd = self.current_gains[motor_id]
            command = MotorCommand()
            command.motor_id = motor_id
            command.torque = 0.0
            command.position = min(upper, max(lower, self.current_pose[motor_id]))
            command.velocity = 0.0
            command.kp = kp
            command.kd = kd
            message.commands.append(command)
        self.publisher.publish(message)
        self.previous_command = dict(self.current_pose)
        self.previous_publish_time = now

    def _tick(self):
        try:
            self._poll_gamepad()
            if self.wait_for_walk_initialized and not self.walk_initialized:
                self._publish()
                self.get_logger().info(
                    "holding idle while waiting for /walk_initialized=true",
                    throttle_duration_sec=2.0)
                return

            elapsed = time.monotonic() - self.state_started
            if self.state == self.STARTUP_DELAY:
                if elapsed >= self.startup_delay:
                    self._enter(self.IDLE)
                    self.get_logger().info("FSM: IDLE; accepting gamepad buttons")

            elif self.state == self.IDLE:
                self.current_pose = dict(self.idle_pose)
                self.current_gains = dict(self.default_gains)

            elif self.state == self.INTERPOLATE_TO_MOTION:
                self.current_pose, self.current_gains, alpha = self._interpolate(elapsed)
                if alpha >= 1.0:
                    self._enter(self.MOTION_PLAY)
                    self.get_logger().info(
                        f"{self.CYAN}>>> PLAY {self.active_motion['name']} "
                        f"profile={self.active_motion['gain_profile']}{self.RESET}")

            elif self.state == self.MOTION_PLAY:
                duration = (
                    (len(self.active_motion["samples"]) - 1) *
                    self.active_motion["period"])
                self.current_pose = self._interpolated_motion_pose(
                    self.active_motion, elapsed)
                self.current_gains = dict(self.active_motion["gains"])
                if elapsed >= duration:
                    self.get_logger().info(
                        f"{self.GREEN}<<< DONE {self.active_motion['name']}{self.RESET}")
                    self._start_interpolation(
                        self.idle_pose, self.default_gains, self.INTERPOLATE_TO_IDLE)

            elif self.state == self.INTERPOLATE_TO_IDLE:
                self.current_pose, self.current_gains, alpha = self._interpolate(elapsed)
                if alpha >= 1.0:
                    self.active_motion = None
                    self._enter(self.IDLE)
                    self.get_logger().info("FSM: IDLE; accepting gamepad buttons")

            self._publish()
        except Exception as error:
            self.get_logger().fatal(str(error))
            self.timer.cancel()

    def destroy_node(self):
        if self.gamepad is not None:
            self.gamepad.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
