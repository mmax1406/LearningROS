import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():


    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    package_name='balance_bot'
    # Locate the world file within the package share directory
    world_file = os.path.join(
        get_package_share_directory(package_name), 
        'worlds', 
        'empty.world'
    )

    # 1. Robot State Publisher (RSP)
    # Renders the URDF into the 'robot_description' parameter
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Gazebo Sim Server/Client Launch
    # Include the Gazebo Sim launch file, provided by the ros_gz_sim package
    # The 'gz_args' now includes the path to your 'empty.world' file and the '-r' flag
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={'gz_args': world_file + ' -r'}.items()
             )

    # 3. Spawner Node (GZ equivalent)
    # Spawns the robot defined in the 'robot_description' parameter.
    create = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot',
                                   '-allow_renaming', 'true'], 
                        output='screen')

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        output='screen'
    )

    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )


    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        create,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
        ros_gz_bridge,
    ])