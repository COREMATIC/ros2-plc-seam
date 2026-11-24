"""Bring up the control side, a PLC, and exactly one target source.

    ros2 launch plc_seam bringup.launch.py source:=random          # no hardware at all
    ros2 launch plc_seam bringup.launch.py source:=sim             # points from a simulator
    ros2 launch plc_seam bringup.launch.py source:=camera          # the real thing
    ros2 launch plc_seam bringup.launch.py plc:=ads plc_ip_address:=... plc_ams_net_id:=... \
        local_ams_net_id:=...                                      # a real Beckhoff PLC

plc:=sim (the default) runs plc_sim, a node that publishes the controllers'
topics and plays the PLC's side of the handshake. plc:=ads runs
ros2_control_node with b»robotized's Beckhoff ADS driver and the three
controllers. The bridge, the robot description and the target sources are
identical in every combination: only the PLC node and the source node change.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command, EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def equals(name, value):
    return IfCondition(EqualsSubstitution(LaunchConfiguration(name), value))


SOURCES = {
    'random': 'target_source_random',
    'sim': 'target_source_sim',
    'camera': 'target_source_camera',
}


def generate_launch_description():
    share = FindPackageShare('plc_seam')
    controllers = PathJoinSubstitution([share, 'config', 'controllers.yaml'])
    description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([share, 'urdf', 'robot.urdf.xacro']),
        ' plc_ip_address:=', LaunchConfiguration('plc_ip_address'),
        ' plc_ams_net_id:=', LaunchConfiguration('plc_ams_net_id'),
        ' local_ams_net_id:=', LaunchConfiguration('local_ams_net_id'),
    ]), value_type=str)
    on_ads = equals('plc', 'ads')

    source_nodes = [
        Node(package='plc_seam', executable=executable, name='target_source',
             output='screen', condition=equals('source', key))
        for key, executable in SOURCES.items()
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'source', default_value='random',
            description='Which target source to run: random | sim | camera'),
        DeclareLaunchArgument(
            'plc', default_value='sim',
            description='sim: a ROS node stands in for the PLC | ads: a Beckhoff PLC over ADS'),
        DeclareLaunchArgument('plc_ip_address', default_value='192.168.0.10'),
        DeclareLaunchArgument('plc_ams_net_id', default_value='192.168.0.10.1.1'),
        DeclareLaunchArgument('local_ams_net_id', default_value='192.168.0.20.1.1'),

        # TF and RViz get the machine's state from /joint_states, whoever publishes it.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),

        # plc:=sim. One node in place of ros2_control_node and its controllers.
        Node(package='plc_seam', executable='plc_sim', name='plc_sim', output='screen',
             condition=equals('plc', 'sim')),

        # plc:=ads. The real binding: the driver plus three controllers, no planner.
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[controllers], output='screen', condition=on_ads),
        # One switch for all three: the handshake must not start before the
        # setpoint controller exists, or the first target never reaches the PLC.
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['joint_state_broadcaster', 'forward_command', 'gpio_command',
                        '--activate-as-group'],
             condition=on_ads),

        Node(package='plc_seam', executable='plc_bridge',
             name='plc_bridge', output='screen'),

        *source_nodes,
    ])
