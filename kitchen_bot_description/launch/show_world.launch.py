import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import xacro

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    # use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    
    pkg_share = get_package_share_directory('kitchen_bot_description')
    # urdf_path = os.path.join(pkg_share, 'urdf', 'kitchen_bot.urdf.xacro')
    world_path = os.path.join(pkg_share, 'world', 'kitchen_world.sdf')

    env_var = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value='/home/aayush/Projects/kitchen_ws/install/kitchen_bot_description/share/kitchen_bot_description/models' +
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )


    # robot_description = xacro.process_file(urdf_path)
    # robot_description = robot_description.toxml()

    declare_world_path = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': ['-r ', world_path],
            'on_exit_shutdown': 'True'
        }.items(),
    )

    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robot_description}]
    # )

    # gz_spawn_entity = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     output='screen',
    #     arguments=['-topic', '/robot_description', '-name',
    #                'kitchen_bot', '-allow_renaming', 'true', '-z', '1.0'],
    # )

    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )


    print(os.environ.get("GZ_SIM_RESOURCE_PATH"))


    return LaunchDescription([
        env_var,
        declare_world_path,
        # robot_state_publisher_node,
        # gz_spawn_entity,
        ros_gz_bridge_node
    ])