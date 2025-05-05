import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # ── your hard-coded paths ───────────────────────────────────────────────
    world_file = '/home/vincentzr/ros2_ws/src/ref_pomdp_neurips23/simulator/worlds/nav1.world'
    robot_file = '/home/vincentzr/ros2_ws/src/ref_pomdp_neurips23/simulator/robots/turtlebot3_burger/model.sdf'
    robot_name = 'turtlebot'

    # ── disable online model database to speed up loading ────────────────────
    env = os.environ.copy()
    env['GAZEBO_MODEL_DATABASE_URI'] = ''

    # 1) start the Gazebo server (with ROS‐2 plugins) via gzserver
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen',
        additional_env=env
    )

    # 2) start the GUI
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        additional_env=env
    )

    # 3) after a short pause, spawn the robot via ros2 run (avoids the logger bug)
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', robot_name,
                    '-file',   robot_file,
                    '-x', '1', '-y', '1', '-z', '0', '-Y', '0'
                ],
                output='screen',
                additional_env=env
            )
        ]
    )

    # 4) finally, your POMDP solver node
    ref_solver = Node(
        package='ref_pomdp_neurips23',
        executable='ref_solver_node',
        name='ref_solver_node',
        output='screen'
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        spawn_robot,
        ref_solver,
    ])

