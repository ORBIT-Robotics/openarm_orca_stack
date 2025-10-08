from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Find the package share directory
    assembly_pkg_share = FindPackageShare('assembly_description')

    # Declare arguments
    use_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start joint_state_publisher_gui'
    )
    
    # Evaluate the assembly Xacro
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([assembly_pkg_share, 'urdf', 'assembly.urdf']),
        ' bimanual:=true hand:=false ee_type:=none'
    ])
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )
    
    # Joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    # RViz node. It finds 'default.rviz' in the 'rviz' share folder.
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


