from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package= 'dual_franka_model',
            executable = 'mujoco_node',
            name = 'mujoco_sim_node',
            parameters = [{'model_path': 'mjcf/mjx_panda.xml'}]
        )
    ])