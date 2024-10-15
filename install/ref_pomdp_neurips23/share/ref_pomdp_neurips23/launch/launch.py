import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Define paths to the world and robot files
    world_file = '/home/vincentzr/ref_pomdp_neurips23/simulator/worlds/DubinMaze/DubinMazeEnvironment.sdf'
    robot_file = '/home/vincentzr/ref_pomdp_neurips23/simulator/robots/Dubin/Dubin.sdf'
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

    # Command to spawn the SDF robot using the gazebo_ros spawn_entity service
    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity',
            f'{{name: "{robot_name}", xml: "{robot_sdf_escaped}", initial_pose: {{position: {{x: 0, y: 0, z: 0}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}}}}}'
        ],
        output='screen'
    )
    
    
 
    # Add the Gazebo, spawn robot, and the ref_solver_node processes to the launch description
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_robot)


    return ld

