"""Official UR5e mock driver + MoveIt + RViz. This launch never connects to a real arm."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    driver = Path(get_package_share_directory("ur_robot_driver"))
    demo = Path(get_package_share_directory("ur5e_pose_control"))
    model_arguments = {
        "name": "ur5e", "ur_type": "ur5e", "robot_ip": "127.0.0.1",
        "use_mock_hardware": "true", "safety_limits": "true",
    }
    # Reuse UR's model, IK solver, joint limits, and OMPL planning settings.
    config = (
        MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
        .robot_description(str(driver / "urdf/ur.urdf.xacro"), mappings=model_arguments)
        .robot_description_semantic("srdf/ur.srdf.xacro", mappings={"name": "ur5e"})
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    # Mock hardware uses the ordinary trajectory controller, not UR speed scaling.
    controllers = config.trajectory_execution["moveit_simple_controller_manager"]
    controllers["controller_names"] = ["joint_trajectory_controller"]
    controllers["joint_trajectory_controller"]["default"] = True
    config.trajectory_execution["trajectory_execution"]["execution_duration_monitoring"] = True

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(driver / "launch/ur_control.launch.py")),
            launch_arguments={
                "ur_type": "ur5e", "robot_ip": "127.0.0.1",
                "use_mock_hardware": "true", "launch_rviz": "false",
                "initial_joint_controller": "joint_trajectory_controller",
                "update_rate_config_file": str(demo / "config/mock_control.yaml"),
            }.items(),
        ),
        Node(package="moveit_ros_move_group", executable="move_group",
             output="screen", parameters=[config.to_dict(), {"use_sim_time": False}]),
        Node(package="rviz2", executable="rviz2", output="screen",
             condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", str(demo / "config/demo.rviz")],
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, config.planning_pipelines,
                         config.joint_limits, {"use_sim_time": False}]),
    ])
