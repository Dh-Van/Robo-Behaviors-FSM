from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robo_behaviors_fsm', # required
            executable='state_machine', # required, name of python script that spins the node
            name='statemachine', # optional, useful for renaming nodes if you want to run multiple of the same node
        ),
        Node(
            package='robo_behaviors_fsm', # required
            executable='drive_circle', # required, name of python script that spins the node
            name='drive_circle', # optional, useful for renaming nodes if you want to run multiple of the same node
        ),
        Node(
            package='robo_behaviors_fsm', # required
            executable='wall_follow', # required, name of python script that spins the node
            name='wall_following', # optional, useful for renaming nodes if you want to run multiple of the same node
        ),
        Node(
            package='robo_behaviors_fsm', # required
            executable='person_following', # required, name of python script that spins the node
            name='person_following', # optional, useful for renaming nodes if you want to run multiple of the same node
        ),
           
    ])
# ros2 launch robo_behaviors_fsm launch.py