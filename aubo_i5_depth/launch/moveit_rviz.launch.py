from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("aubo_i5_depth_camera", package_name="aubo_i5_depth").to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
