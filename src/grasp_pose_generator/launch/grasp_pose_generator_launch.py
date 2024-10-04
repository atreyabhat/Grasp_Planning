from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='grasp_pose_generator',
            executable='grasp_pose_generator_node',
            name='grasp_pose_generator',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': True}
            ]
        )
    ])