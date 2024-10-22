import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Define paths to the world and robot files
    world_file = '/home/vincentzr/ros2_ws/src/ref_pomdp_neurips23/simulator/worlds/DubinMaze/DubinMazeEnvironment.sdf'
    robot_file = '/home/vincentzr/ros2_ws/src/ref_pomdp_neurips23/simulator/robots/Dubin/Dubin.sdf'
    robot_name = 'dubin_robot'

    # Read the SDF file content
    with open(robot_file, 'r') as file:
        robot_sdf = file.read()

    # Escape double quotes for YAML compatibility
    robot_sdf_escaped = robot_sdf.replace('"', '\\"')

    # Create the launch description
    ld = LaunchDescription()

    # Launch Gazebo with the specified world file
    gazebo_launch = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn the robot in Gazebo using the spawn_entity service
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-file', robot_file,
            '-x', '0', '-y', '0', '-z', '0',
            '-Y', '0'
        ],
        output='screen'
    )

    # Launch the ref_solver_node
    ref_solver_node = Node(
        package='ref_pomdp_neurips23',
        executable='ref_solver_node',
        name='ref_solver_node',
        output='screen'
    )

    # Add the Gazebo, spawn robot, and ref_solver_node processes to the launch description
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_robot)
    ld.add_action(ref_solver_node)

    return ld
