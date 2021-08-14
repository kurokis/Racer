import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import subprocess

def is_new_ros_version():
    out = subprocess.Popen(['ros2', 'wtf'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    stdout,stderr = out.communicate()
    s = str(stdout)
    if ("foxy" in s) or ("galactic" in s):
        return True
    else:
        return False

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

    new_ros_version = is_new_ros_version()

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

    if new_ros_version:
        mode_node = Node(
            package='racer',
            executable='mode', # for new versions of ros
        )
    else:
        mode_node = Node(
            package='racer',
            node_executable='mode',
        )

    if new_ros_version:
        keyboard_node = Node(
            package='racer',
            executable='keyboard', # for new versions of ros
            # open another terminal for keyboard control
            prefix='xterm -e',
            output='screen',
        )
    else:
        keyboard_node = Node(
            package='racer',
            node_executable='keyboard',
            # open another terminal for keyboard control
            prefix='xterm -e',
            output='screen',
        )

    if new_ros_version:
        key_ctl_node = Node(
            package='racer',
            executable='key_ctl', # for new versions of ros
        )
    else:
        key_ctl_node = Node(
            package='racer',
            node_executable='key_ctl',
        )

    if new_ros_version:
        joystick_node = Node(
            package='racer',
            executable='joystick', # for new versions of ros
        )
    else:
        joystick_node = Node(
            package='racer',
            node_executable='joystick',
        )

    if new_ros_version:
        joy_ctl_node = Node(
            package='racer',
            executable='joy_ctl', # for new versions of ros
        )
    else:
        joy_ctl_node = Node(
            package='racer',
            node_executable='joy_ctl',
        )

    if new_ros_version:
        nn_ctl_node = Node(
            package='racer',
            executable='nn_ctl', # for new versions of ros
            output='screen',  # print logger info
        )
    else:
        nn_ctl_node = Node(
            package='racer',
            node_executable='nn_ctl',
            output='screen',  # print logger info
        )

    if new_ros_version:
        priority_node = Node(
            package='racer',
            executable='priority', # for new versions of ros
        )
    else:
        priority_node = Node(
            package='racer',
            node_executable='priority',
        )

    if new_ros_version:
        s_motor_node = Node(
            package='racer',
            executable='s_motor', # for new versions of ros
            output='screen',
        )
    else:
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
