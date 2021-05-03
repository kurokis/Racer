from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    keyboard_node = Node(
        package='racer',
        executable='keyboard',
        # open another terminal for keyboard control
        prefix = 'xterm -e',
        output='screen',
    )
    
    key_ctl_node = Node(
        package='racer',
        executable='key_ctl',
    )
    
    s_motor_node = Node(
        package='racer',
        executable='s_motor',
        output='screen',
    )
    
    ld.add_action(keyboard_node)
    ld.add_action(key_ctl_node)
    ld.add_action(s_motor_node)
    
    return ld
