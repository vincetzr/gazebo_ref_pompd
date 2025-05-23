import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, EmitEvent
from launch_ros.actions import Node
from launch.events import Shutdown

def generate_launch_description():
    # Edit here to change the world and robot model
    world_file = '/path_to/ros2_ws/src/ref_pomdp_neurips23/simulator/worlds/nav1.world'
    robot_file = '/path_to/ros2_ws/src/ref_pomdp_neurips23/simulator/robots/turtlebot3_burger/model.sdf'
    robot_name = 'turtlebot'

    # Start the Gazebo server 
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen',
    )

    # Start the GUI
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    # Spawn Robot in Gazebo
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', robot_name,
                    '-file',   robot_file,
                    # Edit here to change the position and orientation of the robot
                    '-x', '6', '-y', '-9', '-z', '0', '-Y', '1.5708'
                ],
                output='screen',
            )
        ]
    )

    # Launch the REFSOLVER node
    ref_solver = Node(
        package='ref_pomdp_neurips23',
        executable='ref_solver_node',
        name='ref_solver_node',
        output='screen',
        on_exit=[EmitEvent(event=Shutdown())]
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        spawn_robot,
        ref_solver,
    ])

