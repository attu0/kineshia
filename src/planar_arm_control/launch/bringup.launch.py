from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Declare launch arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz', default_value='50.0',
        description='Rate at which joint states are published (Hz)'
    )
    control_mode_arg = DeclareLaunchArgument(
        'control_mode', default_value='position',
        description='Mode of the controller (e.g., position, velocity)'
    )
    max_velocity_arg = DeclareLaunchArgument(
        'max_joint_velocity', default_value='1.5',
        description='Max joint velocity for velocity mode (rad/s)'
    )

    # 2. Extract configurations
    publish_rate = LaunchConfiguration('publish_rate_hz')
    control_mode = LaunchConfiguration('control_mode')
    max_velocity = LaunchConfiguration('max_joint_velocity')

    # 3. Define Controller Node
    controller_node = Node(
        package="planar_arm_control",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[{
            "publish_rate_hz": publish_rate,
            "control_mode": control_mode,
            "max_joint_velocity": max_velocity
        }]
    )

    # 4. Define GUI Node
    gui_node = Node(
        package="planar_arm_control",
        executable="gui_node",
        name="gui_node",
        output="screen",
    )

    return LaunchDescription([
        publish_rate_arg,
        control_mode_arg,
        max_velocity_arg,
        controller_node,
        gui_node,
    ])