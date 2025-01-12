from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("aubo_i10_depth_camera", package_name="aubo_i10_depth").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
