cd "$(dirname "$0")" && cd src && colcon build --packages-select racer && . install/setup.bash && ros2 launch racer sim_racer_record.launch.py
