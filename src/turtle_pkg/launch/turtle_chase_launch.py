from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    turtle_environment_node = Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        )
    
    turtle_game_node = Node(
            package='turtle_pkg',
            executable='turtle_chase',
            name='game'
        )
    
    ld.add_action(turtle_environment_node)
    ld.add_action(turtle_game_node)
    return ld