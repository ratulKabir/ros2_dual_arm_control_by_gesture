from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("dual_arm_2d", package_name="control_dual_arm_2d").to_moveit_configs()
    return generate_move_group_launch(moveit_config)
