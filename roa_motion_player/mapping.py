import math


MOTOR_ID_TO_JOINT = {
    0: "left_shoulder_pitch",
    1: "right_shoulder_pitch",
    2: "left_shoulder_roll",
    3: "right_shoulder_roll",
    4: "left_arm_yaw",
    5: "right_arm_yaw",
    6: "left_arm_pitch",
    7: "right_arm_pitch",
    9: "torso_yaw",
    10: "left_hip_pitch",
    11: "right_hip_pitch",
    12: "left_hip_roll",
    13: "right_hip_roll",
    14: "left_hip_yaw",
    15: "right_hip_yaw",
    16: "left_knee_pitch",
    17: "right_knee_pitch",
}

RIGHT_ARM_JOINT_TO_ID = {
    "right_shoulder_pitch": 1,
    "right_shoulder_roll": 3,
    "right_arm_yaw": 5,
    "right_arm_pitch": 7,
}

LEFT_ARM_JOINT_TO_ID = {
    "left_shoulder_pitch": 0,
    "left_shoulder_roll": 2,
    "left_arm_yaw": 4,
    "left_arm_pitch": 6,
}

ARM_JOINT_TO_ID = {**RIGHT_ARM_JOINT_TO_ID, **LEFT_ARM_JOINT_TO_ID}

# IDs configured by robstride_hardware_interface. ID 8 is unused.
CONFIGURED_MOTOR_IDS = tuple(list(range(0, 8)) + list(range(9, 24)))

# PacketManager gains are authoritative for IDs 9..21. Upper-body joints do
# not yet have controller-specific gains, so they use torso_yaw (75, 2.5).
TORSO_FALLBACK_GAIN = (75.0, 2.5)
MOTOR_GAINS = {
    **{motor_id: TORSO_FALLBACK_GAIN for motor_id in range(0, 8)},
    9: (75.0, 2.5),
    10: (150.0, 24.722), 11: (150.0, 24.722),
    12: (200.0, 26.387), 13: (200.0, 26.387),
    14: (100.0, 3.419), 15: (100.0, 3.419),
    16: (150.0, 8.654), 17: (150.0, 8.654),
    18: (15.75, 2.5), 19: (15.75, 2.5),
    20: (15.75, 2.5), 21: (15.75, 2.5),
    22: TORSO_FALLBACK_GAIN, 23: TORSO_FALLBACK_GAIN,
}

POSITION_LIMITS = {motor_id: (-math.pi, math.pi) for motor_id in CONFIGURED_MOTOR_IDS}
POSITION_LIMITS.update({
    9: (-1.570796, 1.570796),
    10: (-1.570796, 1.570796), 11: (-1.570796, 1.570796),
    12: (-0.523599, 1.570796), 13: (-1.570796, 0.523599),
    14: (-1.570796, 1.570796), 15: (-1.570796, 1.570796),
    16: (-1.570796, 1.570796), 17: (-1.570796, 1.570796),
})
