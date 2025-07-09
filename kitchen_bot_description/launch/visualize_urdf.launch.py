import os
import xacro
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    env_var = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value='/home/aayush/Projects/kitchen_ws/install/kitchen_bot_description/share/kitchen_bot_description/models' +
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    pkg_share = get_package_share_directory('kitchen_bot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'kitchen_bot_with_arm.urdf.xacro')
    world_path = os.path.join(pkg_share, 'world', 'kitchen_world.sdf')
    config_path = os.path.join(pkg_share, 'config', 'diffdrive_controller.yaml')

    declare_world_path = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': ['-r ', world_path],
            'on_exit_shutdown': 'True'
        }.items(),
    )

    doc = xacro.process_file(urdf_path)
    robot_description_config = doc.toxml()

    publish_robot_state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config}]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description', '-name',
                   'kitchen_bot', '-allow_renaming', 'true', '-z', '1.0'],
    )

    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )

    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mobile_base_controller', '--param-file', config_path],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        env_var,
        declare_world_path,
        publish_robot_state,
        gz_spawn_entity,
        ros_gz_bridge_node,
        joint_state_broadcaster_spawner,
        diffdrive_controller_spawner
    ])