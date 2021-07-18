import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import subprocess


def generate_launch_description():
    # Error prevention: automatically stop gzserver (server process for gazebo)
    # if it is already running
    subprocess.run(["killall","-9","gzserver"])

    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_file_name = 'walls2.world'
    pkg_dir = get_package_share_directory('racer')

    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models')

    world = os.path.join(pkg_dir, 'worlds', world_file_name)
    launch_file_dir = os.path.join(pkg_dir, 'launch')

    # gazebo = gzserver (simulation) + gzclient (GUI)
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # Run only the simulation part of gazebo. Visualization to be done on rviz.
    # gzserver = ExecuteProcess(
    #    cmd=['gzserver', '--verbose', world, '-s', 'libgazebo_ros_init.so',
    #        '-s', 'libgazebo_ros_factory.so'],
    #    output='screen',
    # )

    # Run rviz with preset configuration
    # rviz = ExecuteProcess(
    #    cmd=['rviz2','-d','./src/rviz_config.rviz'],
    # )

    mode_node = Node(
        package='racer',
        node_executable='mode',
    )

    keyboard_node = Node(
        package='racer',
        node_executable='keyboard',
        # open another terminal for keyboard control
        prefix='xterm -e',
        output='screen',
    )

    key_ctl_node = Node(
        package='racer',
        node_executable='key_ctl',
    )

    joystick_node = Node(
        package='racer',
        node_executable='joystick',
    )

    joy_ctl_node = Node(
        package='racer',
        node_executable='joy_ctl',
    )

    nn_ctl_node = Node(
        package='racer',
        node_executable='nn_ctl',
        output='screen',  # print logger info
    )

    priority_node = Node(
        package='racer',
        node_executable='priority',
    )

    s_motor_node = Node(
        package='racer',
        node_executable='s_motor',
        output='screen',
    )

    ld.add_action(gazebo)
    # ld.add_action(gzserver)
    # ld.add_action(rviz)
    ld.add_action(mode_node)
    ld.add_action(keyboard_node)
    ld.add_action(key_ctl_node)
    ld.add_action(joystick_node)
    ld.add_action(joy_ctl_node)
    ld.add_action(nn_ctl_node)
    ld.add_action(priority_node)
    ld.add_action(s_motor_node)

    return ld
