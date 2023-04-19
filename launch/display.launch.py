
import os 
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    ## Find Files
    pkg_name = 'diff_nav_robot'
    pkg_share = get_package_share_directory(pkg_name)
    model_default_path = os.path.join(pkg_share, 'description/robot_description.urdf')
    rviz_config_default_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    world_default_path = os.path.join(pkg_share, 'world/my_world.sdf')
    filter_config = os.path.join(pkg_share, 'config/ekf.yaml')

    ## Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rviz_config')

    ## Arguments
    model_desc_arg = DeclareLaunchArgument(
        name='model',
        default_value=model_default_path,
        description='Absolute path to robot urdf file')
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time')
    rviz_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=rviz_config_default_path,
        description='Absolute path to rviz config file')


    ## Commands
    robot_desc = Command(['xacro ', model])
    gazebo_world = ExecuteProcess(
                                cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_default_path], 
                                output='screen')

    
    ## Nodes
    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share, 'launch','robot_state_pub.launch.py')])
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_node',
        output='screen'
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[filter_config, {'use_sim_time': use_sim_time}]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen',
        arguments=['-d', rviz_config]
    )

    spawn_entity = Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'diff_nav_bot', '-topic', 'robot_description'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_online_async_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        model_desc_arg,
        use_sim_time_arg,
        rviz_arg,
        robot_state_publisher_node,
        rviz_node,
        gazebo_world,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        robot_localization_node,
        slam_toolbox_node
    ])