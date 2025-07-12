from typing import List

# MoveIt Planning Groups
MOVE_GROUP_ARM: str = "arm_1"
MOVE_GROUP_GRIPPER: str = "gripper_1"

# Gripper States
OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]

def joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "joint1_1",
        prefix + "joint2_1",
        prefix + "joint3_1",
        prefix + "joint4_1",
        prefix + "joint5_1",
        prefix + "joint6_1",
    ]

def base_link_name(prefix: str = "") -> str:
    return prefix + "base_link"

def end_effector_name(prefix: str = "") -> str:
    # You can replace this with the link attached to your TCP if needed
    return prefix + "palm_link_1"

def gripper_joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "left_finger_joint_1",
        prefix + "right_finger_joint_1",
    ]
