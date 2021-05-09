import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_file_name = 'walls2.world'
    pkg_dir = get_package_share_directory('racer')
 
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models')
 
    world = os.path.join(pkg_dir, 'worlds', world_file_name)
    launch_file_dir = os.path.join(pkg_dir, 'launch')
 
    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', 
            '-s', 'libgazebo_ros_factory.so'],
            output='screen')
    #gazebo = ExecuteProcess(
    #        cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
    #        output='screen')
 
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
    
    joy_ctl_node = Node(
        package='racer',
        node_executable='joy_ctl',
    )

    s_motor_node = Node(
        package='racer',
        node_executable='s_motor',
        output='screen',
    )
    
    ld.add_action(gazebo)
    ld.add_action(keyboard_node)
    ld.add_action(key_ctl_node)
    ld.add_action(joy_ctl_node)
    ld.add_action(s_motor_node)
    
    return ld
