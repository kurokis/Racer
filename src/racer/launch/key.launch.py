from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    key_ctl_node = Node(
        package='racer',
            executable='key_ctl',
    )
    
    #keyboard_node = Node(
    #    package='racer',
    #        executable='keyboard',
    #        output='screen',
    #)
    
    ld.add_action(key_ctl_node)
    #ld.add_action(keyboard_node)
    
    return ld
