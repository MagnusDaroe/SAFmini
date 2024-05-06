import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
         Node(
            package='saf',
            executable='processtime_node.py',
            name='process',
            output='screen', # to see the output of the node
        ),
        Node(
            package='saf',
            executable='server_node.py',
            name='server',
            output='screen', # to see the output of the node
        ),
       
    ])

