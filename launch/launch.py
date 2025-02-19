import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Define paths to the world and robot files
    world_file = '/home/vincentzr/ros2_ws/src/ref_pomdp_neurips23/simulator/worlds/turtle_worlds/turtlebot3_dqn_stage1.world'
    robot_file = '/home/vincentzr/ros2_ws/src/ref_pomdp_neurips23/simulator/robots/turtlebot3_burger/model.sdf'
    robot_name = 'turtlebot'

    '''# Read the SDF file content
    with open(robot_file, 'r') as file:
        robot_sdf = file.read()

    # Escape double quotes for YAML compatibility
    robot_sdf_escaped = robot_sdf.replace('"', '\\"')'''

    # Create the launch description
    ld = LaunchDescription()

    # Launch Gazebo with the specified world file
    gz_sim_launch = ExecuteProcess(
        cmd=['gz', 'sim', '--verbose', world_file],
        output='screen'
    )
    
    # Spawn the robot in Gazebo using the spawn_entity service
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'default',  # Make sure 'default' is the correct world name in your SDF
            '-file', robot_file,
            '-name', robot_name,
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )

    # Bridge topics between ROS 2 and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/turtlebot/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        output='screen'
    )
    
    # Launch the ref_solver_node
    '''ref_solver_node = Node(
        package='ref_pomdp_neurips23',
        executable='ref_solver_node',
        name='ref_solver_node',
        output='screen'
    )'''
    
    # Add your dummy.py node
    dummy_node = Node(
        package='ref_pomdp_neurips23', 
        executable='dummy',           
        name='dummy_node',
        output='screen'
    )

    # Add the Gazebo, spawn robot, and ref_solver_node processes to the launch description
    ld.add_action(gz_sim_launch)
    ld.add_action(spawn_robot)
    #ld.add_action(ref_solver_node)
    ld.add_action(dummy_node)
    
    return ld
