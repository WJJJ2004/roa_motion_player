#!/usr/bin/env python3
"""Generic adapter from the existing hardware command to URDF JointState."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from roa_interfaces.msg import MotorCommandArray, RsuTarget

from .mapping import MOTOR_ID_TO_JOINT


ANKLE_JOINTS = (
    "left_ankle_roll", "left_ankle_pitch",
    "right_ankle_roll", "right_ankle_pitch",
)


class HwCmdToJointState(Node):
    def __init__(self):
        super().__init__("hw_cmd_to_joint_state")
        command_topic = str(self.declare_parameter(
            "command_topic", "/hardware_interface/command").value)
        joint_state_topic = str(self.declare_parameter("joint_state_topic", "/joint_states").value)
        rsu_topic = str(self.declare_parameter("rsu_target_topic", "/rsu/target").value)
        rate = float(self.declare_parameter("publish_rate_hz", 100.0).value)
        self.positions = {name: 0.0 for name in MOTOR_ID_TO_JOINT.values()}
        self.positions.update({name: 0.0 for name in ANKLE_JOINTS})
        command_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(MotorCommandArray, command_topic, self._on_command, command_qos)
        self.create_subscription(RsuTarget, rsu_topic, self._on_rsu_target, 10)
        self.publisher = self.create_publisher(JointState, joint_state_topic, 10)
        self.create_timer(1.0 / rate, self._publish)

    def _on_command(self, message):
        for command in message.commands:
            joint = MOTOR_ID_TO_JOINT.get(int(command.motor_id))
            if joint is not None and math.isfinite(command.position):
                # MotorCommand positions and feedback are already in the model
                # convention; RobStrideMotor applies protocol sign conversion.
                self.positions[joint] = float(command.position)

    def _on_rsu_target(self, message):
        values = {
            "left_ankle_roll": message.left_roll,
            "left_ankle_pitch": message.left_pitch,
            "right_ankle_roll": message.right_roll,
            "right_ankle_pitch": message.right_pitch,
        }
        for joint, value in values.items():
            if math.isfinite(value):
                self.positions[joint] = float(value)

    def _publish(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.positions)
        message.position = [self.positions[name] for name in message.name]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = HwCmdToJointState()
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
