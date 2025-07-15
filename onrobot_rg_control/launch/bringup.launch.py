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
    control_arg = DeclareLaunchArgument(
        'control',
        default_value=TextSubstitution(text='modbus')
    )
    
    offset_arg = DeclareLaunchArgument(
        'offset',
        default_value='5'
    )
    
    isaac_joint_states_arg = DeclareLaunchArgument(
        'isaac_joint_states',
        default_value='/isaac_joint_states'
    )
    
    isaac_joint_commands_arg = DeclareLaunchArgument(
        'isaac_joint_commands',
        default_value='/isaac_joint_commands'
    )
    
    server_node = Node(
        package='onrobot_rg_control',
        executable='OnRobotRGControllerServer',
        name='OnRobotRGControllerServer',
        output='screen',
        arguments=[],
        parameters=[{
            '/onrobot/control': LaunchConfiguration('control'),
            '/onrobot/ip': LaunchConfiguration('ip'),
            '/onrobot/port': LaunchConfiguration('port'),
            '/onrobot/changer_addr': LaunchConfiguration('changer_addr'),
            '/onrobot/gripper': LaunchConfiguration('gripper'),
            '/onrobot/offset': LaunchConfiguration('offset'),
            '/onrobot/isaac_joint_states': LaunchConfiguration('isaac_joint_states'),
            '/onrobot/isaac_joint_commands': LaunchConfiguration('isaac_joint_commands'),
        }],
    )
    
    # Launching all the nodes
    return LaunchDescription(
        [
            ip_arg,
            port_arg,
            gripper_arg,
            changer_addr_arg,
            control_arg,
            offset_arg,
            isaac_joint_states_arg,
            isaac_joint_commands_arg,
            
            server_node,
        ]
    )