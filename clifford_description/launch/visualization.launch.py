# Libraries for file handling
import os
from ament_index_python import get_package_share_directory
# Libraries for node launching
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch import LaunchDescription

share_pkg_filepath = get_package_share_directory("clifford_description")

robot_state_publisher_launch_filepath = os.path.join(
    share_pkg_filepath, 
    "launch", 
    "robot_state_publisher.launch.py"
)

rviz_config_filepath = os.path.join(
    share_pkg_filepath, 
    "config", 
    "visualization.rviz"
)

def generate_launch_description():
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_state_publisher_launch_filepath]),
        launch_arguments= {
            'use_sim_time': 'true',
            'use_ros2_control': 'true'
        }.items()
    )

    rviz_cmd = ExecuteProcess(
        cmd= ["rviz2", "-d", rviz_config_filepath]
    )

    joint_state_publisher_gui_node = Node(
        package= "joint_state_publisher_gui",
        executable= "joint_state_publisher_gui",
    )

    nodes_to_run = [
        robot_state_publisher_launch, 
        rviz_cmd, 
        joint_state_publisher_gui_node
    ]
    return LaunchDescription(nodes_to_run)
       
