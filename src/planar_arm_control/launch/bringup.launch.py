#!/usr/bin/env python3

'''
*****************************************************************************************
*
*                    ===========================================
*                       planar_arm_control
*                    ===========================================
*
*  ROS 2 launch file for the planar arm controller and telemetry GUI.
*
*****************************************************************************************
'''

# Author:           Atharv Mudse
# Mail ID:          atharvmudse@gmail.com
# Filename:         bringup.launch.py
# Functions:        generate_launch_description
# Nodes:            controller_node, gui_node

################### IMPORT MODULES #######################

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


##################### FUNCTION DEFINITION #######################

def generate_launch_description():
    '''
    Description: Creates the launch description for the planar arm controller
                 and GUI nodes.

    Returns:
        LaunchDescription: Launch description containing the declared arguments
                           and ROS 2 nodes.
    '''

    ##################### LAUNCH ARGUMENTS #######################

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='50.0',
        description='Rate at which joint states are published (Hz)'
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='position',
        description='Mode of the controller (position or velocity)'
    )

    max_velocity_arg = DeclareLaunchArgument(
        'max_joint_velocity',
        default_value='1.5',
        description='Maximum joint velocity for velocity mode (rad/s)'
    )

    ##################### CONFIGURATIONS #######################

    publish_rate = LaunchConfiguration('publish_rate_hz')
    control_mode = LaunchConfiguration('control_mode')
    max_velocity = LaunchConfiguration('max_joint_velocity')

    ##################### CONTROLLER NODE #######################

    controller_node = Node(
        package='planar_arm_control',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[{
            'publish_rate_hz': publish_rate,
            'control_mode': control_mode,
            'max_joint_velocity': max_velocity
        }]
    )

    ##################### GUI NODE #######################

    gui_node = Node(
        package='planar_arm_control',
        executable='gui_node',
        name='gui_node',
        output='screen'
    )

    ##################### LAUNCH DESCRIPTION #######################

    return LaunchDescription([
        publish_rate_arg,
        control_mode_arg,
        max_velocity_arg,
        controller_node,
        gui_node
    ])


if __name__ == '__main__':
    generate_launch_description()
