from typing import List

# MoveIt Planning Groups
MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "gripper"

# Gripper States
OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]

def joint_names(prefix: str = "") -> List[str]:
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

def end_effector_name(prefix: str = "") -> str:
    # You can replace this with the link attached to your TCP if needed
    return prefix + "palm_link"

def gripper_joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "left_finger_joint",
        prefix + "right_finger_joint",
    ]
