import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directories
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    elfin_description_pkg = get_package_share_directory('elfin_description')
    elfin_gazebo_pkg = get_package_share_directory('elfin_gazebo')

    # Define the launch arguments
    controller_arg = DeclareLaunchArgument(
        'controller', default_value='gravity_comp_controller',
        description='Controller to load'
    )

    # Include the Gazebo empty world launch file
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r', }.items(),
    )

    # # Gazebo Sim
    # pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # gazebo_empty_world = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    #     launch_arguments={'gz_args': 'empty.sdf -r', }.items(),
    # )    

    # Robot description (URDF model loaded via xacro)
    robot_description_command = Command([
        FindExecutable(name='xacro'),
        ' ', os.path.join(elfin_description_pkg, 'urdf', 'elfin3.urdf.xacro')
    ])

    robot_description = {'robot_description': robot_description_command}

    # Spawn the URDF model into Gazebo
    urdf_spawner = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'elfin3', '-topic', 'robot_description'],
        output='screen'
    )

    # Load the robot controllers
    load_controllers = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_controller', LaunchConfiguration('controller')],
        namespace='/elfin',
        output='screen'
    )

    # Use PathJoinSubstitution to join the path for controller parameters
    controller_params_path = PathJoinSubstitution([
        elfin_gazebo_pkg, 'config', LaunchConfiguration('controller'), '.yaml'
    ])

    load_controller_params = Node(
        package='ros2_control_node',
        executable='spawner',
        output='screen',
        arguments=[LaunchConfiguration('controller')],
        parameters=[controller_params_path]
    )

    # Robot state publisher
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='/elfin',
        output='screen',
        parameters=[robot_description]
    )

    # Launch description
    return LaunchDescription([
        controller_arg,
        gazebo_launch,
        urdf_spawner,
        state_publisher,
        load_controllers,
        load_controller_params
    ])
