import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gokart_launch_pkg_prefix = get_package_share_directory('autoware_launch')
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_stack')

    # param files
    '''
    Joystick config files (DS4, F29, AT9S)
    '''
    joy_teleop_config = os.path.join(
            f1tenth_launch_pkg_prefix,
            'config',
            'joy_teleop.yaml'
    )
    # a29s does not have a config file

    '''
    Mux config files    
    '''
    mux_config = os.path.join(
            get_package_share_directory('f1tenth_stack'),
            'config',
            'mux.yaml'
    )
    at9s_param_file = os.path.join(
            gokart_launch_pkg_prefix, 'config/params/at9s_teleop.yaml')

    '''
    VESC config
    '''
    vesc_config = os.path.join(
            f1tenth_launch_pkg_prefix,
            'config',
            'vesc.yaml'
    )

    """ Launch Arguments """
    joy_la = DeclareLaunchArgument(
            'joy_config',
            default_value=joy_teleop_config,
            description='Descriptions for joy and joy_teleop configs')
    vesc_la = DeclareLaunchArgument(
            'vesc_config',
            default_value=vesc_config,
            description='Descriptions for vesc configs')
    mux_la = DeclareLaunchArgument(
            'mux_config',
            default_value=mux_config,
            description='Descriptions for ackermann mux configs')
    with_joy_param = DeclareLaunchArgument(
            'with_joy',
            default_value='True',
            description='Launch joystick_interface in addition to other nodes'
    )
    with_at9s_param = DeclareLaunchArgument(
            'with_at9s_joy',
            default_value='True',
            description='Launch at9s_interface in addition to other nodes'
    )

    """ Load launch descriptions. """
    ld = LaunchDescription([joy_la, vesc_la, mux_la, with_joy_param, with_at9s_param])

    """Nodes"""
    # joystick driver node
    joy_node = Node(
            package='joy',  # 'joy_linux'
            executable='joy_node',  # 'joy_linux_node'
            name='joy',
            output='screen',
            parameters=[LaunchConfiguration('joy_config')],
            condition=IfCondition(LaunchConfiguration('with_joy'))
    )

    at9s_joy_node = Node(
            package='at9s_joy',  # 'joy_linux'
            executable='at9s_joy',  # 'joy_linux_node'
            name='at9s_joy',
            output='screen',
            parameters=[at9s_param_file],
            condition=IfCondition(LaunchConfiguration('with_at9s_param'))
    )

    '''teleop/mux nodes'''
    joy_teleop_node = Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            parameters=[LaunchConfiguration('joy_config')]
    )

    at9s_teleop_node = Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='at9s_teleop',
            parameters=[at9s_param_file]
    )

    ackermann_to_vesc_node = Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            # namespace='vehicle',
            parameters=[LaunchConfiguration('vesc_config')]
    )

    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        # namespace='vehicle',
        # output='screen',
        parameters=[LaunchConfiguration('vesc_config')]
    )

    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        # namespace='vehicle',
        # output='screen',
        parameters=[LaunchConfiguration('vesc_config')],
    )

    throttle_interpolator_node = Node(
            package='f1tenth_stack',
            executable='throttle_interpolator',
            name='throttle_interpolator',
            parameters=[LaunchConfiguration('vesc_config')]
    )
    ackermann_mux_node = Node(
            package='ackermann_mux',
            executable='ackermann_mux',
            name='ackermann_mux',
            parameters=[LaunchConfiguration('mux_config')],
            remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )

    # static_tf_node = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='static_baselink_to_laser',
    #         arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
    # )

    # todo: setup sensors
    # todo: add teleop

    # add nodes to the launch description
    ld.add_action(joy_node)
    ld.add_action(with_joy_param)
    ld.add_action(joy_teleop_node)
    ld.add_action(at9s_joy_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    # ld.add_action(throttle_interpolator_node)
    ld.add_action(ackermann_mux_node)
    # ld.add_action(static_tf_node)

    return ld


