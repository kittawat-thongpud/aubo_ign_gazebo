from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("aubo_i5_depth_camera", package_name="aubo_i5_depth").to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
