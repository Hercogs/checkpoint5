from launch import LaunchDescription, actions, substitutions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():

    obstacle_arg = actions.DeclareLaunchArgument('obstacle', default_value='0.3')
    degrees_arg = actions.DeclareLaunchArgument('degrees', default_value='90')
    final_approach_arg = actions.DeclareLaunchArgument('final_approach', default_value='False')

    preapproach_node = Node(
        package='attach_shelf',
        executable='pre_approach_v2_node',
        output='screen',
        name='pre_approach_v2',
        parameters=[{'use_sim_time': True}, {'obstacle': substitutions.LaunchConfiguration('obstacle')},\
                    {'degrees': substitutions.LaunchConfiguration('degrees')},\
                    {'final_approach': substitutions.LaunchConfiguration('final_approach')}])

    approach_service = Node(
        package='attach_shelf',
        executable='approach_service_server_node',
        output='screen',
        name='approach_service',
        parameters=[{'use_sim_time': True}]
        )

    
    # RVIZ configuration
    rviz_config_dir = os.path.join(get_package_share_directory(
        'attach_shelf'), 'rviz', 'config.rviz')

    
    # RVIZ node for visualizing
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])


    # create and return launch description object
    return LaunchDescription(
        [
            obstacle_arg,
            degrees_arg,
            final_approach_arg,
            preapproach_node,
            approach_service,
            rviz_node
        ]
    )