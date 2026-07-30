from typing import List

import rclpy
from rclpy.node import Node

from mit_msgs.msg import MITJointCommand, MITJointCommands, MITLowState


class HoldCommandPublisher(Node):
    """Generate 26-motor hold commands at 50 Hz without touching the SDK by default."""

    def __init__(self) -> None:
        super().__init__("s4_hold_command_publisher")

        self.declare_parameter("state_topic", "/human_lower_state")
        self.declare_parameter("dryrun_topic", "/s4/dryrun/human_lower_command")
        self.declare_parameter("sdk_command_topic", "/human_lower_command")
        self.declare_parameter("enable_sdk_command", False)
        self.declare_parameter("expected_motor_count", 26)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("hold_kp", 5.0)
        self.declare_parameter("hold_kd", 0.2)
        self.declare_parameter("arm_hold_kp", 5.0)
        self.declare_parameter("arm_hold_kd", 0.2)
        self.declare_parameter("effort", 0.0)

        self._state_topic = self.get_parameter("state_topic").value
        self._dryrun_topic = self.get_parameter("dryrun_topic").value
        self._sdk_command_topic = self.get_parameter("sdk_command_topic").value
        self._enable_sdk_command = bool(self.get_parameter("enable_sdk_command").value)
        self._expected_motor_count = int(self.get_parameter("expected_motor_count").value)
        self._hold_kp = float(self.get_parameter("hold_kp").value)
        self._hold_kd = float(self.get_parameter("hold_kd").value)
        self._arm_hold_kp = float(self.get_parameter("arm_hold_kp").value)
        self._arm_hold_kd = float(self.get_parameter("arm_hold_kd").value)
        self._effort = float(self.get_parameter("effort").value)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self._command_topic = (
            self._sdk_command_topic if self._enable_sdk_command else self._dryrun_topic
        )
        if self._enable_sdk_command:
            self.get_logger().warn(
                "enable_sdk_command=true: commands will be published to "
                f"{self._sdk_command_topic}"
            )
        else:
            self.get_logger().info(
                "SDK command output is disabled. Publishing dry-run commands to "
                f"{self._dryrun_topic}"
            )

        self._last_positions: List[float] = []
        self._last_velocities: List[float] = []
        self._state_ready = False
        self._warned_waiting = False

        self._state_sub = self.create_subscription(
            MITLowState, self._state_topic, self._state_callback, 10
        )
        self._command_pub = self.create_publisher(
            MITJointCommands, self._command_topic, 10
        )
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._timer_callback)

        self.get_logger().info(
            "Configured 50 Hz hold command generator: "
            f"state_topic={self._state_topic}, command_topic={self._command_topic}, "
            f"expected_motor_count={self._expected_motor_count}"
        )

    def _state_callback(self, msg: MITLowState) -> None:
        positions = list(msg.joint_states.position)
        velocities = list(msg.joint_states.velocity)

        if len(positions) != self._expected_motor_count:
            self.get_logger().warn(
                "Ignoring state with "
                f"{len(positions)} positions; expected {self._expected_motor_count}",
                throttle_duration_sec=2.0,
            )
            self._state_ready = False
            return

        if velocities and len(velocities) != len(positions):
            self.get_logger().warn(
                "State velocity length does not match position length; using zeros",
                throttle_duration_sec=2.0,
            )
            velocities = [0.0] * len(positions)
        elif not velocities:
            velocities = [0.0] * len(positions)

        self._last_positions = positions
        self._last_velocities = velocities
        self._state_ready = True

    def _timer_callback(self) -> None:
        if not self._state_ready:
            if not self._warned_waiting:
                self.get_logger().info(
                    f"Waiting for {self._expected_motor_count}-motor state on "
                    f"{self._state_topic}"
                )
                self._warned_waiting = True
            return

        out = MITJointCommands()
        out.stamp = self.get_clock().now().to_msg()
        out.commands = []

        for index, position in enumerate(self._last_positions):
            command = MITJointCommand()
            is_arm_motor = 12 <= index <= 25
            command.kp = self._arm_hold_kp if is_arm_motor else self._hold_kp
            command.kd = self._arm_hold_kd if is_arm_motor else self._hold_kd
            command.pos = float(position)
            command.vel = 0.0
            command.eff = self._effort
            out.commands.append(command)

        self._command_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HoldCommandPublisher()
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
