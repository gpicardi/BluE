import os, time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'silver'
    file_subpath = 'description/silver.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Use config file for rviz
    base_path = os.path.realpath(get_package_share_directory('silver'))
    rviz_path=base_path+'/config/rviz_config_file.rviz'

    # Configure nodes
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name),'launch','rsp_silver.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    base_path_w = os.path.realpath(get_package_share_directory('silver'))
    custom_world=base_path_w+'/worlds/cylinder.world'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'world': custom_world}.items(),
        )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic','robot_description','-entity', 'silver', '-z', '0.7'],
        output='screen')
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )

    forward_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller", "--controller-manager", "/controller_manager", "--inactive"],
    )

    forward_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_effort_controller", "--controller-manager", "/controller_manager", "--inactive"],
    )

    joint_trajectory_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_position_controller", "--controller-manager", "/controller_manager", "--inactive"],
    )

    pid_position_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'pid_position_controller'],
        output='screen'
    )

    """pid_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pid_position_controller", "--controller-manager", "/controller_manager", "active"],
    )"""

    locomotion_spawner = Node(
        package="silver",
        executable="locomotion.py",
        arguments=[],
    )

    delayed_locomotion_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[locomotion_spawner],
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", str(rviz_path)],
    )

    delayed_rviz_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delayed_pid_position_controller_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[pid_position_controller_spawner],
            )
        )

    # Run the node
    return LaunchDescription([
        gazebo,
        rsp,
        spawn_entity,
        joint_state_broadcaster_spawner,
        delayed_pid_position_controller_spawner,
        #forward_position_controller_spawner,
        #forward_velocity_controller_spawner,
        #forward_effort_controller_spawner,
        #joint_trajectory_position_controller_spawner,
        delayed_rviz_node,
        #delayed_locomotion_spawner
    ])


