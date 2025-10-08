from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    assembly_pkg_share = FindPackageShare('assembly_description')
    openarm_pkg_share = FindPackageShare('openarm_description')
    connectors_pkg_share = FindPackageShare('connectors_description')
    orcahand_pkg_share = FindPackageShare('orcahand_description')

    left_hand_urdf = PathJoinSubstitution([orcahand_pkg_share, 'models', 'urdf', 'orcahand_left.urdf'])
    right_hand_urdf = PathJoinSubstitution([orcahand_pkg_share, 'models', 'urdf', 'orcahand_right.urdf'])

    use_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start joint_state_publisher_gui'
    )

    robot_description = Command([
        TextSubstitution(text='xacro '),
        PathJoinSubstitution([assembly_pkg_share, 'urdf', 'assembly.urdf']),
        TextSubstitution(text=' bimanual:=true'),
        TextSubstitution(text=' hand:=false'),
        TextSubstitution(text=' ee_type:=none'),
        TextSubstitution(text=' openarm_description_path:='),
        openarm_pkg_share,
        TextSubstitution(text=' connectors_description_path:='),
        connectors_pkg_share,
        TextSubstitution(text=' orcahand_description_path:='),
        orcahand_pkg_share,
        TextSubstitution(text=' orcahand_left_urdf:='),
        left_hand_urdf,
        TextSubstitution(text=' orcahand_right_urdf:='),
        right_hand_urdf
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([assembly_pkg_share, 'rviz', 'default.rviz'])]
    )

    return LaunchDescription([
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
