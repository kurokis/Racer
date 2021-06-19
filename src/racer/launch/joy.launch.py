from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Error prevention: automatically stop gzserver (server process for gazebo)
    # if it is already running
    subprocess.run(["killall","-9","gzserver"])
    
    ld = LaunchDescription()
    
    keyboard_node = Node(
        package='racer',
        node_executable='keyboard',
        # open another terminal for keyboard control
        prefix = 'xterm -e',
        output='screen',
    )
    
    key_ctl_node = Node(
        package='racer',
        node_executable='key_ctl',
    )
    
    joy_ctl_node= Node(
        package='racer',
        node_executable='joy_ctl',
    )
    
    s_motor_node = Node(
        package='racer',
        node_executable='s_motor',
        output='screen',
    )
    
    ld.add_action(keyboard_node)
    ld.add_action(key_ctl_node)
    ld.add_action(s_motor_node)
    
    return ld
