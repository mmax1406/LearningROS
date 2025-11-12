import os
import xacro
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import launch

package_name = "balance_bot"
robot_name = "balance_bot"
xacroRelativePath = 'description/robot.urdf.xacro'
rvizRelativePath = 'config/config.rviz'
ros2controllerRelativePath = 'config/my_controllers.yaml'

def generate_launch_description():
   
    # Paths
    pkg_share = launch_ros.substitutions.FindPackageShare(package_name).find(package_name)
    urdf_file = os.path.join(pkg_share, xacroRelativePath)
    controller_yaml = os.path.join(pkg_share, ros2controllerRelativePath)
    rviz_file  = os.path.join(pkg_share, rvizRelativePath) 

    robot_desc = xacro.process_file(urdf_file).toxml()
    robot_description = {'robot_description': robot_desc}

    declared_args = []
    declared_args.append(
        launch.actions.DeclareLaunchArgument(name="gui",default_value="true",
                                             description="Start the RViz2 GUI"
                ))


    gui = LaunchConfiguration("gui")
    # Launch arguments
    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_ros.substitutions.FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", " -r -v 3 empty.sdf")],
        condition = launch.conditions.IfCondition(gui)
    )

    gazebo_headless = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_ros.substitutions.FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", "--headless-rendering -s -r -v 3 empty.sdf")],
        condition = launch.conditions.IfCondition(gui)
    )

    gz_bridge_node = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    gz_spawn_entity = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'robot_system_position',
            '-allow_renaming','true'
        ],
        output='screen'
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=['-d', rviz_file],
        output='screen'
    )

    control_node = launch_ros.actions.Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='rviz2',
        parameters=[controller_yaml],
        output='both'
    )
    
    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )
    
    controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', "--param-file", controller_yaml],
    )
    
    nodeList = [
        gazebo,
        gazebo_headless,
        gz_bridge_node,
        gz_spawn_entity,
        robot_state_publisher_node,
        rviz_node,
        control_node,
        joint_state_broadcaster_spawner,
        controller_spawner
    ]

    return launch.LaunchDescription(declared_args + nodeList)