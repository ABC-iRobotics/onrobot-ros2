from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    
    ip_arg = DeclareLaunchArgument(
        'ip',
        default_value=TextSubstitution(text='192.168.1.1')
    )
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='502'
    )
    gripper_arg = DeclareLaunchArgument(
        'gripper',
        default_value=TextSubstitution(text='rg6')
    )
    changer_addr_arg = DeclareLaunchArgument(
        'changer_addr',
        default_value='65'
    )
    dummy_arg = DeclareLaunchArgument(
        'dummy',
        default_value='True'
    )
    offset_arg = DeclareLaunchArgument(
        'offset',
        default_value='5'
    )
    
    status_node = Node(
        package='onrobot_rg_control',
        executable='OnRobotRGStatusListener',
        name='OnRobotRGStatusListener',
        output='screen',
        arguments=[],
        parameters=[],
    )
    
    tcp_node = Node(
        package='onrobot_rg_control',
        executable='OnRobotRGTcpNode',
        name='OnRobotRGTcpNode',
        output='screen',
        arguments=[],
        parameters=[{
            '/onrobot/ip': LaunchConfiguration('ip'),
            '/onrobot/port': LaunchConfiguration('port'),
            '/onrobot/gripper': LaunchConfiguration('gripper'),
            '/onrobot/changer_addr': LaunchConfiguration('changer_addr'),
            '/onrobot/dummy': LaunchConfiguration('dummy'),
            '/onrobot/offset': LaunchConfiguration('offset'),
        }],
    )
    
    # Launching all the nodes
    return LaunchDescription(
        [
            ip_arg,
            port_arg,
            gripper_arg,
            changer_addr_arg,
            dummy_arg,
            offset_arg,
            status_node,
            tcp_node,
        ]
    )