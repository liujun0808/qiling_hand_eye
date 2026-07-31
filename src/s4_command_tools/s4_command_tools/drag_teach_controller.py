from __future__ import annotations

import math
import os
from typing import Dict, List, Optional

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from mit_msgs.msg import MITJointCommand, MITJointCommands, MITLowState
from rclpy.node import Node
from sensor_msgs.msg import Joy

from s4_handeye_calibration.s4_model import ARM_JOINT_NAMES, S4Kinematics


def _finite_values(values: List[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def _clamp(value: float, limit: float) -> float:
    if limit <= 0.0:
        return value
    return max(-limit, min(limit, value))


def _default_joint_config_path() -> str:
    try:
        share_dir = get_package_share_directory("s4_command_tools")
        return os.path.join(share_dir, "config", "drag_teach_joints.yaml")
    except PackageNotFoundError:
        return os.path.join(
            os.getcwd(), "src", "s4_command_tools", "config", "drag_teach_joints.yaml"
        )


def _value(config: Dict[str, object], key: str, fallback: float) -> float:
    return float(config.get(key, fallback))


class DragTeachController(Node):
    """Low-stiffness 50 Hz drag-teach command generator for the 26-motor MIT bridge."""

    ARM_START_INDEX = 12
    ARM_COUNT = 14
    JOINTS_PER_ARM = 7

    def __init__(self) -> None:
        super().__init__("s4_drag_teach_controller")

        self.declare_parameter("state_topic", "/human_lower_state")
        self.declare_parameter("dryrun_topic", "/s4/dryrun/drag_teach_command")
        self.declare_parameter("sdk_command_topic", "/human_lower_command")
        self.declare_parameter("enable_sdk_command", False)
        self.declare_parameter("control_enabled", False)
        self.declare_parameter("expected_motor_count", 26)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("allow_non_50hz", False)
        self.declare_parameter("state_timeout_sec", 0.1)
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("joint_config_path", "")
        self.declare_parameter("gravity_compensation", True)
        self.declare_parameter("gravity_scale", 0.0)
        self.declare_parameter("gravity_ramp_time_sec", 2.0)
        self.declare_parameter("activation_hold_time_sec", 1.0)
        self.declare_parameter("safety_hold_error_limit_rad", 0.25)
        self.declare_parameter("teach_mode", "drag_hold")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("left_drag_button_index", 0)
        self.declare_parameter("right_drag_button_index", 1)
        self.declare_parameter("joy_timeout_sec", 0.25)
        self.declare_parameter("arm_kp", 0.0)
        self.declare_parameter("arm_kd", 0.35)
        self.declare_parameter("hold_arm_kp", 5.0)
        self.declare_parameter("hold_arm_kd", 0.6)
        self.declare_parameter("arm_effort_limit", 8.0)
        self.declare_parameter("leg_hold_kp", 10.0)
        self.declare_parameter("leg_hold_kd", 0.3)
        self.declare_parameter("latch_leg_positions", True)
        self.declare_parameter("max_abs_position_rad", 6.3)
        self.declare_parameter("max_abs_velocity_rad_s", 8.0)
        self.declare_parameter("publish_passive_on_fault", True)

        self._state_topic = self.get_parameter("state_topic").value
        self._dryrun_topic = self.get_parameter("dryrun_topic").value
        self._sdk_command_topic = self.get_parameter("sdk_command_topic").value
        self._enable_sdk_command = bool(self.get_parameter("enable_sdk_command").value)
        self._expected_motor_count = int(self.get_parameter("expected_motor_count").value)
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._allow_non_50hz = bool(self.get_parameter("allow_non_50hz").value)
        self._state_timeout_sec = float(self.get_parameter("state_timeout_sec").value)
        self._gravity_compensation = bool(self.get_parameter("gravity_compensation").value)
        self._gravity_scale = float(self.get_parameter("gravity_scale").value)
        self._gravity_ramp_time_sec = float(self.get_parameter("gravity_ramp_time_sec").value)
        self._activation_hold_time_sec = float(self.get_parameter("activation_hold_time_sec").value)
        self._safety_hold_error_limit = float(
            self.get_parameter("safety_hold_error_limit_rad").value
        )
        self._teach_mode = self.get_parameter("teach_mode").value
        self._joy_topic = self.get_parameter("joy_topic").value
        self._drag_button_indices = [
            int(self.get_parameter("left_drag_button_index").value),
            int(self.get_parameter("right_drag_button_index").value),
        ]
        self._joy_timeout_sec = float(self.get_parameter("joy_timeout_sec").value)
        self._arm_kp = float(self.get_parameter("arm_kp").value)
        self._arm_kd = float(self.get_parameter("arm_kd").value)
        self._hold_arm_kp = float(self.get_parameter("hold_arm_kp").value)
        self._hold_arm_kd = float(self.get_parameter("hold_arm_kd").value)
        self._arm_effort_limit = float(self.get_parameter("arm_effort_limit").value)
        self._leg_hold_kp = float(self.get_parameter("leg_hold_kp").value)
        self._leg_hold_kd = float(self.get_parameter("leg_hold_kd").value)
        self._latch_leg_positions = bool(self.get_parameter("latch_leg_positions").value)
        self._max_abs_position_rad = float(self.get_parameter("max_abs_position_rad").value)
        self._max_abs_velocity_rad_s = float(self.get_parameter("max_abs_velocity_rad_s").value)
        self._publish_passive_on_fault = bool(
            self.get_parameter("publish_passive_on_fault").value
        )
        self._joint_config_path = self.get_parameter("joint_config_path").value

        if self._expected_motor_count != 26:
            raise ValueError("drag_teach_controller currently requires expected_motor_count=26")
        if self._publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        if not self._allow_non_50hz and abs(self._publish_rate_hz - 50.0) > 1e-6:
            raise ValueError("Upper-limb command rate is fixed at 50 Hz; keep publish_rate_hz=50.0")
        if self._state_timeout_sec <= 0.0:
            raise ValueError("state_timeout_sec must be positive")
        if self._gravity_ramp_time_sec < 0.0:
            raise ValueError("gravity_ramp_time_sec must be non-negative")
        if self._activation_hold_time_sec < 0.0:
            raise ValueError("activation_hold_time_sec must be non-negative")
        if self._safety_hold_error_limit < 0.0:
            raise ValueError("safety_hold_error_limit_rad must be non-negative")
        if self._teach_mode not in ("transparent", "drag_hold"):
            raise ValueError("teach_mode must be 'transparent' or 'drag_hold'")
        if any(index < 0 for index in self._drag_button_indices):
            raise ValueError("left/right_drag_button_index must be non-negative")
        if self._drag_button_indices[0] == self._drag_button_indices[1]:
            raise ValueError("left/right_drag_button_index must select different buttons")
        if self._joy_timeout_sec <= 0.0:
            raise ValueError("joy_timeout_sec must be positive")

        self._kinematics = S4Kinematics(self.get_parameter("urdf_path").value)
        self._load_joint_config()
        self._command_topic = (
            self._sdk_command_topic if self._enable_sdk_command else self._dryrun_topic
        )

        self._positions: List[float] = []
        self._velocities: List[float] = []
        self._leg_hold_positions: Optional[List[float]] = None
        self._arm_hold_positions: Optional[List[float]] = None
        self._arm_states = ["DRAG"] * self.ARM_COUNT
        self._joy_received = False
        self._joy_buttons_valid = False
        self._drag_buttons_pressed = [False, False]
        self._clutch_armed = [False, False]
        self._clutch_active = [False, False]
        self._last_joy_time_sec = 0.0
        self._warned_joy_timeout = False
        self._last_control_enabled = False
        self._active_since_time_sec: Optional[float] = None
        self._faulted = False
        self._fault_reason = ""
        self._last_state_time_sec = 0.0
        self._state_ready = False
        self._warned_waiting = False
        self._warned_disabled = False

        self._state_sub = self.create_subscription(
            MITLowState, self._state_topic, self._state_callback, 10
        )
        self._joy_sub = self.create_subscription(Joy, self._joy_topic, self._joy_callback, 10)
        self._command_pub = self.create_publisher(MITJointCommands, self._command_topic, 10)
        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._timer_callback)

        if self._enable_sdk_command:
            self.get_logger().warn(
                "SDK command topic selected. Commands are still blocked until "
                "control_enabled=true."
            )
        else:
            self.get_logger().info(f"SDK command disabled; publishing dry-run to {self._dryrun_topic}")

        self.get_logger().info(
            "Drag-teach controller configured: "
            f"rate={self._publish_rate_hz} Hz, state_topic={self._state_topic}, "
            f"command_topic={self._command_topic}, teach_mode={self._teach_mode}, "
            f"joy_topic={self._joy_topic}, "
            f"left_drag_button_index={self._drag_button_indices[0]}, "
            f"right_drag_button_index={self._drag_button_indices[1]}, "
            f"gravity_compensation={self._gravity_compensation}, "
            f"joint_config_path={self._joint_config_path or _default_joint_config_path()}"
        )

    def _load_joint_config(self) -> None:
        path = self._joint_config_path or _default_joint_config_path()
        self._joint_config_path = path
        defaults = {
            "drag_kp": self._arm_kp,
            "drag_kd": self._arm_kd,
            "hold_kp": self._hold_arm_kp,
            "hold_kd": self._hold_arm_kd,
            "gravity_scale": 1.0,
            "effort_limit": self._arm_effort_limit,
            "safety_hold_error_limit_rad": self._safety_hold_error_limit,
            "gravity_sign": 1.0,
            "max_velocity_rad_s": 2.0,
        }
        data: Dict[str, object] = {}
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
            self.get_logger().info(f"Loaded per-joint drag-teach config: {path}")
        else:
            self.get_logger().warn(f"Joint config not found, using scalar parameters: {path}")

        arm_defaults = data.get("arm_defaults", {}) or {}
        joints = data.get("joints", {}) or {}

        self._joint_drag_kp: List[float] = []
        self._joint_drag_kd: List[float] = []
        self._joint_hold_kp: List[float] = []
        self._joint_hold_kd: List[float] = []
        self._joint_gravity_scale: List[float] = []
        self._joint_effort_limit: List[float] = []
        self._joint_safety_hold_error_limit: List[float] = []
        self._joint_gravity_sign: List[float] = []
        self._joint_min_position: List[float] = []
        self._joint_max_position: List[float] = []
        self._joint_max_velocity: List[float] = []
        self._joint_release_prediction_time: List[float] = []
        self._joint_release_prediction_limit: List[float] = []

        for joint_name in ARM_JOINT_NAMES:
            model_lower, model_upper = self._model_position_limits(joint_name)
            joint_defaults = dict(defaults)

            config = dict(joint_defaults)
            config.update(arm_defaults)
            config.update(joints.get(joint_name, {}) or {})

            self._joint_drag_kp.append(_value(config, "drag_kp", joint_defaults["drag_kp"]))
            self._joint_drag_kd.append(_value(config, "drag_kd", joint_defaults["drag_kd"]))
            self._joint_hold_kp.append(_value(config, "hold_kp", joint_defaults["hold_kp"]))
            self._joint_hold_kd.append(_value(config, "hold_kd", joint_defaults["hold_kd"]))
            self._joint_gravity_scale.append(
                _value(config, "gravity_scale", joint_defaults["gravity_scale"])
            )
            self._joint_gravity_sign.append(
                _value(config, "gravity_sign", joint_defaults["gravity_sign"])
            )
            self._joint_effort_limit.append(
                _value(config, "effort_limit", joint_defaults["effort_limit"])
            )
            self._joint_safety_hold_error_limit.append(
                _value(
                    config,
                    "safety_hold_error_limit_rad",
                    joint_defaults["safety_hold_error_limit_rad"],
                )
            )
            self._joint_min_position.append(model_lower)
            self._joint_max_position.append(model_upper)
            self._joint_max_velocity.append(
                _value(config, "max_velocity_rad_s", joint_defaults["max_velocity_rad_s"])
            )
            prediction_time = _value(config, "release_prediction_time_sec", 0.0)
            prediction_limit = _value(config, "release_prediction_limit_rad", 0.0)
            if prediction_time < 0.0 or prediction_limit < 0.0:
                raise ValueError(
                    f"{joint_name} release prediction parameters must be non-negative"
                )
            self._joint_release_prediction_time.append(prediction_time)
            self._joint_release_prediction_limit.append(prediction_limit)

    def _model_position_limits(self, joint_name: str) -> tuple[float, float]:
        joint_id = self._kinematics.model.getJointId(joint_name)
        q_index = self._kinematics.model.joints[joint_id].idx_q
        lower = float(self._kinematics.model.lowerPositionLimit[q_index])
        upper = float(self._kinematics.model.upperPositionLimit[q_index])
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            return -math.inf, math.inf
        return lower, upper

    def _state_callback(self, msg: MITLowState) -> None:
        positions = [float(value) for value in msg.joint_states.position]
        velocities = [float(value) for value in msg.joint_states.velocity]

        if len(positions) != self._expected_motor_count:
            self.get_logger().warn(
                f"Ignoring state with {len(positions)} positions; expected {self._expected_motor_count}",
                throttle_duration_sec=2.0,
            )
            self._state_ready = False
            return
        if velocities and len(velocities) != len(positions):
            self.get_logger().warn(
                "Velocity length does not match position length; using zeros",
                throttle_duration_sec=2.0,
            )
            velocities = [0.0] * len(positions)
        elif not velocities:
            velocities = [0.0] * len(positions)
        if not _finite_values(positions) or not _finite_values(velocities):
            self.get_logger().warn("Ignoring non-finite state values", throttle_duration_sec=2.0)
            self._state_ready = False
            return

        if self._max_abs_position_rad > 0.0:
            if max(abs(value) for value in positions) > self._max_abs_position_rad:
                self.get_logger().warn("Ignoring state beyond max_abs_position_rad", throttle_duration_sec=2.0)
                self._state_ready = False
                return
        if self._max_abs_velocity_rad_s > 0.0:
            if max(abs(value) for value in velocities) > self._max_abs_velocity_rad_s:
                self.get_logger().warn("Ignoring state beyond max_abs_velocity_rad_s", throttle_duration_sec=2.0)
                self._state_ready = False
                return

        if self._leg_hold_positions is None:
            self._leg_hold_positions = positions[: self.ARM_START_INDEX]
        if self._arm_hold_positions is None:
            self._arm_hold_positions = positions[
                self.ARM_START_INDEX : self.ARM_START_INDEX + self.ARM_COUNT
            ]

        self._positions = positions
        self._velocities = velocities
        self._last_state_time_sec = self.get_clock().now().nanoseconds * 1e-9
        self._state_ready = True

    def _joy_callback(self, msg: Joy) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        self._joy_received = True
        self._last_joy_time_sec = now
        highest_button_index = max(self._drag_button_indices)
        if highest_button_index >= len(msg.buttons):
            self._joy_buttons_valid = False
            self._drag_buttons_pressed = [False, False]
            self.get_logger().error(
                f"Joy message has {len(msg.buttons)} buttons; "
                f"configured A/B indices {self._drag_button_indices} are invalid",
                throttle_duration_sec=2.0,
            )
            return
        self._joy_buttons_valid = True
        self._drag_buttons_pressed = [
            bool(msg.buttons[index]) for index in self._drag_button_indices
        ]

    def _timer_callback(self) -> None:
        if not self._state_ready:
            if not self._warned_waiting:
                self.get_logger().info(f"Waiting for 26-motor state on {self._state_topic}")
                self._warned_waiting = True
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_state_time_sec > self._state_timeout_sec:
            self.get_logger().warn("State timeout; command publication paused", throttle_duration_sec=1.0)
            return

        control_enabled = bool(self.get_parameter("control_enabled").value)
        self._handle_control_transition(control_enabled, now)

        if self._enable_sdk_command and not control_enabled:
            if not self._warned_disabled:
                self.get_logger().warn("control_enabled=false; not publishing SDK commands")
                self._warned_disabled = True
            return
        self._warned_disabled = False

        if self._faulted:
            self.get_logger().error(
                "Safety fault active"
                f"{f': {self._fault_reason}' if self._fault_reason else ''}. "
                "Set control_enabled=false, check the robot, then re-enable.",
                throttle_duration_sec=1.0,
            )
            self._publish_passive_fault_command()
            return

        self._refresh_runtime_parameters()
        fault_reason = self._arm_motion_safety_fault()
        if fault_reason:
            self._faulted = True
            self._fault_reason = fault_reason
            self.get_logger().error(f"Safety fault: {fault_reason}")
            self._publish_passive_fault_command()
            return
        self._update_drag_hold_state(now)
        if self._hold_error_exceeds_safety_limit():
            self._faulted = True
            self._fault_reason = "Arm moved too far from latched hold pose"
            self.get_logger().error(self._fault_reason)
            self._publish_passive_fault_command()
            return
        command = self._build_command()
        self._command_pub.publish(command)

    def _handle_control_transition(self, control_enabled: bool, now: float) -> None:
        if control_enabled and not self._last_control_enabled:
            self._arm_hold_positions = self._arm_positions()
            self._leg_hold_positions = self._positions[: self.ARM_START_INDEX]
            self._arm_states = ["HOLD"] * self.ARM_COUNT
            self._clutch_armed = [False, False]
            self._clutch_active = [False, False]
            self._warned_joy_timeout = False
            self._active_since_time_sec = now
            self._faulted = False
            self._fault_reason = ""
            self.get_logger().warn(
                "control_enabled=true: latched current arm pose, entering HOLD. "
                "Release Xbox A and B once to arm the independent drag clutches."
            )
        elif not control_enabled and self._last_control_enabled:
            self._active_since_time_sec = None
            self._clutch_armed = [False, False]
            self._clutch_active = [False, False]
            self._faulted = False
            self._fault_reason = ""
            self.get_logger().warn("control_enabled=false: command publication disabled")
        self._last_control_enabled = control_enabled

    def _refresh_runtime_parameters(self) -> None:
        self._gravity_scale = float(self.get_parameter("gravity_scale").value)
        self._gravity_ramp_time_sec = float(self.get_parameter("gravity_ramp_time_sec").value)
        self._activation_hold_time_sec = float(self.get_parameter("activation_hold_time_sec").value)
        self._safety_hold_error_limit = float(
            self.get_parameter("safety_hold_error_limit_rad").value
        )
        self._arm_kp = float(self.get_parameter("arm_kp").value)
        self._arm_kd = float(self.get_parameter("arm_kd").value)
        self._hold_arm_kp = float(self.get_parameter("hold_arm_kp").value)
        self._hold_arm_kd = float(self.get_parameter("hold_arm_kd").value)
        self._arm_effort_limit = float(self.get_parameter("arm_effort_limit").value)
        self._publish_passive_on_fault = bool(
            self.get_parameter("publish_passive_on_fault").value
        )

    def _arm_positions(self) -> List[float]:
        return self._positions[self.ARM_START_INDEX : self.ARM_START_INDEX + self.ARM_COUNT]

    def _arm_velocities(self) -> List[float]:
        return self._velocities[self.ARM_START_INDEX : self.ARM_START_INDEX + self.ARM_COUNT]

    def _effective_gravity_scale(self) -> float:
        if self._active_since_time_sec is None or self._gravity_ramp_time_sec <= 0.0:
            return self._gravity_scale
        now = self.get_clock().now().nanoseconds * 1e-9
        ramp = _clamp((now - self._active_since_time_sec) / self._gravity_ramp_time_sec, 1.0)
        return self._gravity_scale * ramp

    def _in_activation_hold(self, now: float) -> bool:
        if self._active_since_time_sec is None:
            return False
        return now - self._active_since_time_sec < self._activation_hold_time_sec

    def _hold_error_exceeds_safety_limit(self) -> bool:
        if self._safety_hold_error_limit <= 0.0 or self._arm_hold_positions is None:
            return False
        for index, (current, target) in enumerate(
            zip(self._arm_positions(), self._arm_hold_positions)
        ):
            if self._arm_states[index] != "HOLD":
                continue
            limit = self._joint_safety_hold_error_limit[index]
            if limit > 0.0 and abs(current - target) > limit:
                self.get_logger().error(
                    f"{ARM_JOINT_NAMES[index]} exceeded safety hold error limit"
                )
                return True
        return False

    def _arm_motion_safety_fault(self) -> str:
        arm_positions = self._arm_positions()
        arm_velocities = self._arm_velocities()
        for index, (position, velocity) in enumerate(zip(arm_positions, arm_velocities)):
            joint_name = ARM_JOINT_NAMES[index]
            lower = self._joint_min_position[index]
            upper = self._joint_max_position[index]
            if math.isfinite(lower) and position <= lower:
                return (
                    f"{joint_name} reached URDF lower limit "
                    f"({position:.3f} <= {lower:.3f} rad)"
                )
            if math.isfinite(upper) and position >= upper:
                return (
                    f"{joint_name} reached URDF upper limit "
                    f"({position:.3f} >= {upper:.3f} rad)"
                )
            max_velocity = self._joint_max_velocity[index]
            if max_velocity > 0.0 and abs(velocity) >= max_velocity:
                return (
                    f"{joint_name} velocity too high "
                    f"({velocity:.3f} rad/s >= {max_velocity:.3f} rad/s)"
                )
        return ""

    def _update_drag_hold_state(self, now: float) -> None:
        if self._teach_mode != "drag_hold":
            self._arm_states = ["DRAG"] * self.ARM_COUNT
            self._arm_hold_positions = self._arm_positions()
            return

        if self._in_activation_hold(now):
            self._arm_states = ["HOLD"] * self.ARM_COUNT
            return

        joy_fresh = (
            self._joy_received
            and self._joy_buttons_valid
            and now - self._last_joy_time_sec <= self._joy_timeout_sec
        )
        arm_positions = self._arm_positions()
        arm_velocities = self._arm_velocities()

        if not joy_fresh:
            if any(self._clutch_active):
                self.get_logger().warn(
                    "Joy input timed out or became invalid; active arm clutches entering HOLD"
                )
            elif not self._warned_joy_timeout:
                self.get_logger().warn(
                    f"Waiting for valid Joy input on {self._joy_topic}; arms remain in HOLD"
                )
            self._warned_joy_timeout = True
            clutch_requested = [False, False]
        else:
            self._warned_joy_timeout = False
            for side, button_name in enumerate(("A", "B")):
                if not self._drag_buttons_pressed[side] and not self._clutch_armed[side]:
                    self._clutch_armed[side] = True
                    arm_name = "left" if side == 0 else "right"
                    self.get_logger().info(
                        f"Xbox {button_name} released; {arm_name}-arm drag clutch armed"
                    )
            clutch_requested = [
                self._clutch_armed[side] and self._drag_buttons_pressed[side]
                for side in range(2)
            ]

        if self._arm_hold_positions is None:
            self._arm_hold_positions = list(arm_positions)

        for side, (button_name, arm_name) in enumerate(
            (("A", "left"), ("B", "right"))
        ):
            start = side * self.JOINTS_PER_ARM
            end = start + self.JOINTS_PER_ARM
            if clutch_requested[side]:
                if not self._clutch_active[side]:
                    self.get_logger().warn(
                        f"Xbox {button_name} pressed: {arm_name} arm entering DRAG"
                    )
                for index in range(start, end):
                    self._arm_states[index] = "DRAG"
                    self._arm_hold_positions[index] = arm_positions[index]
            else:
                if self._clutch_active[side]:
                    reason = (
                        f"Xbox {button_name} released"
                        if joy_fresh
                        else "Joy input unavailable"
                    )
                    self._latch_side_release_positions(
                        side, arm_positions, arm_velocities
                    )
                    self._set_side_state(side, "HOLD")
                    self.get_logger().warn(
                        f"{reason}: latched current {arm_name}-arm pose, entering HOLD immediately"
                    )

        self._clutch_active = clutch_requested
        if not joy_fresh:
            self._clutch_armed = [False, False]

    def _side_joint_range(self, side: int) -> range:
        start = side * self.JOINTS_PER_ARM
        return range(start, start + self.JOINTS_PER_ARM)

    def _set_side_state(self, side: int, state: str) -> None:
        for index in self._side_joint_range(side):
            self._arm_states[index] = state

    def _latch_side_release_positions(
        self,
        side: int,
        arm_positions: List[float],
        arm_velocities: List[float],
    ) -> None:
        if self._arm_hold_positions is None:
            self._arm_hold_positions = list(arm_positions)
        for index in self._side_joint_range(side):
            position = arm_positions[index]
            velocity = arm_velocities[index]
            prediction_time = self._joint_release_prediction_time[index]
            prediction_limit = self._joint_release_prediction_limit[index]
            correction = (
                _clamp(velocity * prediction_time, prediction_limit)
                if prediction_time > 0.0 and prediction_limit > 0.0
                else 0.0
            )
            target = position + correction
            target = max(
                self._joint_min_position[index],
                min(self._joint_max_position[index], target),
            )
            self._arm_hold_positions[index] = target

            if prediction_time > 0.0 and prediction_limit > 0.0:
                self.get_logger().warn(
                    f"{ARM_JOINT_NAMES[index]} release latch: "
                    f"q={position:.4f}, dq={velocity:.4f}, "
                    f"correction={target - position:+.4f}, q_hold={target:.4f}"
                )

    def _build_command(self) -> MITJointCommands:
        arm_tau_g = [0.0] * self.ARM_COUNT
        if self._gravity_compensation:
            arm_tau_g = self._kinematics.arm_gravity_torques(self._positions)
        effective_gravity_scale = self._effective_gravity_scale()

        out = MITJointCommands()
        out.stamp = self.get_clock().now().to_msg()
        out.commands = []

        for index, position in enumerate(self._positions):
            motor = MITJointCommand()
            if self.ARM_START_INDEX <= index < self.ARM_START_INDEX + self.ARM_COUNT:
                arm_index = index - self.ARM_START_INDEX
                state = self._arm_states[arm_index]
                if state == "HOLD":
                    motor.kp = self._joint_hold_kp[arm_index]
                    motor.kd = self._joint_hold_kd[arm_index]
                    motor.pos = float(self._arm_hold_positions[arm_index])
                else:
                    motor.kp = self._joint_drag_kp[arm_index]
                    motor.kd = self._joint_drag_kd[arm_index]
                    motor.pos = float(position)
                motor.vel = 0.0
                motor.eff = _clamp(
                    effective_gravity_scale
                    * self._joint_gravity_scale[arm_index]
                    * self._joint_gravity_sign[arm_index]
                    * arm_tau_g[arm_index],
                    self._joint_effort_limit[arm_index],
                )
            else:
                motor.kp = self._leg_hold_kp
                motor.kd = self._leg_hold_kd
                if self._latch_leg_positions and self._leg_hold_positions is not None:
                    motor.pos = float(self._leg_hold_positions[index])
                else:
                    motor.pos = float(position)
                motor.vel = 0.0
                motor.eff = 0.0
            out.commands.append(motor)

        return out

    def _build_passive_fault_command(self) -> MITJointCommands:
        out = MITJointCommands()
        out.stamp = self.get_clock().now().to_msg()
        out.commands = []

        for index, position in enumerate(self._positions):
            motor = MITJointCommand()
            if self.ARM_START_INDEX <= index < self.ARM_START_INDEX + self.ARM_COUNT:
                motor.kp = 0.0
                motor.kd = 0.0
                motor.pos = float(position)
                motor.vel = 0.0
                motor.eff = 0.0
            else:
                motor.kp = self._leg_hold_kp
                motor.kd = self._leg_hold_kd
                if self._latch_leg_positions and self._leg_hold_positions is not None:
                    motor.pos = float(self._leg_hold_positions[index])
                else:
                    motor.pos = float(position)
                motor.vel = 0.0
                motor.eff = 0.0
            out.commands.append(motor)

        return out

    def _publish_passive_fault_command(self) -> None:
        if not self._publish_passive_on_fault:
            return
        self._command_pub.publish(self._build_passive_fault_command())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DragTeachController()
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
