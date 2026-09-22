from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Declare launch arguments to expose parameters to the command line
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='50.0',
        description='Rate at which joint states are published (Hz)'
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='position',
        description='Mode of the controller (e.g., position, trajectory)'
    )

    # 2. Extract the configuration values
    publish_rate = LaunchConfiguration('publish_rate_hz')
    control_mode = LaunchConfiguration('control_mode')

    # 3. Define the Controller Node
    controller_node = Node(
        package="planar_arm_control",
        executable="controller_node", # Make sure this matches your setup.py entry point
        name="controller_node",
        output="screen",
        parameters=[{
            "publish_rate_hz": publish_rate,
            "control_mode": control_mode
        }]
    )

    # 4. Define the GUI Node
    gui_node = Node(
        package="planar_arm_control",
        executable="gui_node",        # Make sure this matches your setup.py entry point
        name="gui_node",
        output="screen",
    )

    return LaunchDescription([
        publish_rate_arg,
        control_mode_arg,
        controller_node,
        gui_node,
    ])