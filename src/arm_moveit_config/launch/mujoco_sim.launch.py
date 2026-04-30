"""
mujoco_sim.launch.py

Launch the full arm stack (MoveIt move_group + ros2_control + RViz) with the
MuJoCo physics-simulation hardware backend.

Usage:
    ros2 launch arm_moveit_config mujoco_sim.launch.py

The MuJoCo MJCF model path defaults to
    <install>/share/mujoco_arm_hardware/mujoco/arm.xml
and can be overridden with:
    ros2 launch arm_moveit_config mujoco_sim.launch.py \\
        mujoco_model_path:=/path/to/custom_arm.xml

What this launch file does NOT start (compared to the real-hardware launch):
  - handeye_publisher  (no physical camera in simulation)
  - Serial-port hardware interface (MuJoCo backend is used instead)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    mujoco_model_path_arg = DeclareLaunchArgument(
        "mujoco_model_path",
        default_value=os.path.join(
            get_package_share_directory("mujoco_arm_hardware"),
            "mujoco",
            "arm.xml",
        ),
        description="Absolute path to the MuJoCo MJCF model file for the arm.",
    )
    mujoco_model_path = LaunchConfiguration("mujoco_model_path")

    # ── MoveIt configuration ──────────────────────────────────────────────────
    # Pass use_sim:=true so arm.ros2_control.xacro selects MujocoArmHardware,
    # and forward mujoco_model_path into the MJCF <param>.
    moveit_config = (
        MoveItConfigsBuilder("arm", package_name="arm_moveit_config")
        .robot_description(
            file_path="config/arm.urdf.xacro",
            mappings={
                "use_sim": "true",
                "mujoco_model_path": mujoco_model_path,
            },
        )
        .robot_description_semantic(file_path="config/arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # ── Path to simulation-specific controller config ─────────────────────────
    ros2_controllers_sim_path = os.path.join(
        get_package_share_directory("arm_moveit_config"),
        "config",
        "ros2_controllers_sim.yaml",
    )

    # ── RViz config ───────────────────────────────────────────────────────────
    rviz_config_path = os.path.join(
        get_package_share_directory("arm_moveit_config"),
        "config",
        "moveit.rviz",
    )

    # ── Nodes ─────────────────────────────────────────────────────────────────

    # ros2_control node loads the MuJoCo hardware plugin via the robot_description
    # (which was rendered with use_sim:=true above).
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_sim_path,
        ],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription(
        [
            mujoco_model_path_arg,
            ros2_control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
            move_group_node,
            rviz_node,
        ]
    )
