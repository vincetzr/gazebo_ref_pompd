from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 'simulator/worlds/world.sdf', '--ros-args'],
            output='screen'
        ),

        # Spawn the robot with sensor 
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', '/home/vincentzr/ref_pomdp_neurips23/simulator/robot/KukaModel/KukaModel.sdf',
                '-entity', 'robot'
            ],
            output='screen'
        ),
    ])

