#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/tmp/ttyUR',
        description='Serial port path (virtual port created by socat)'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='1000000',
        description='Serial baud rate (1000000 or 115200)'
    )
    
    parity_arg = DeclareLaunchArgument(
        'parity',
        default_value='E',
        description='Parity: E(Even), O(Odd), N(None)'
    )
    
    slave_id_arg = DeclareLaunchArgument(
        'slave_id',
        default_value='65',
        description='Modbus slave ID'
    )
    
    polling_rate_arg = DeclareLaunchArgument(
        'polling_rate_hz',
        default_value='50.0',
        description='State polling rate in Hz'
    )
    
    # Get package share directory
    pkg_share = FindPackageShare('onrobot_gripper_driver')
    
    # Parameters
    config_file = PathJoinSubstitution([pkg_share, 'config', 'gripper_params.yaml'])
    
    # Gripper node
    gripper_node = Node(
        package='onrobot_gripper_driver',
        executable='onrobot_gripper_node',
        name='onrobot_gripper_node',
        output='screen',
        parameters=[
            config_file,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'parity': LaunchConfiguration('parity'),
                'slave_id': LaunchConfiguration('slave_id'),
                'polling_rate_hz': LaunchConfiguration('polling_rate_hz'),
            }
        ]
    )
    
    # Delay gripper node startup by 2 seconds to ensure socat is ready
    delayed_gripper_node = TimerAction(
        period=2.0,
        actions=[gripper_node]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        parity_arg,
        slave_id_arg,
        polling_rate_arg,
        delayed_gripper_node
    ])
