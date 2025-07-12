from typing import List

# MoveIt Planning Groups
MOVE_GROUP_ARM: str = "right_arm"
MOVE_GROUP_GRIPPER: str = "right_gripper"

# Gripper States
OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.03, 0.03]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]

def joint_names(prefix: str = "right_") -> List[str]:
    return [
        prefix + "joint1",
        prefix + "joint2",
        prefix + "joint3",
        prefix + "joint4",
        prefix + "joint5",
        prefix + "joint6",
    ]

def base_link_name(prefix: str = "") -> str:
    return prefix + "base_link"

def end_effector_name(prefix: str = "right_") -> str:
    return prefix + "link6"

def gripper_joint_names(prefix: str = "right_") -> List[str]:
    return [
        prefix + "left_finger_joint",
        prefix + "right_finger_joint",
    ]
