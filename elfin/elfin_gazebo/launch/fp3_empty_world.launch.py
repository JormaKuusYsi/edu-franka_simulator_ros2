import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Define paths
    elfin_description_path = os.path.join(os.path.expanduser('~'), 
        'elfin_ros2_ws/src/edu-franka_simulator_ros2/elfin/elfin_description')

    franka_description_path = os.path.join(os.path.expanduser('~'),
        'franka_ros2_ws/src/franka_description')
    
    # Specify the path to the URDF file
    urdf_path = os.path.join(elfin_description_path, 'urdf', 'elfin3.urdf.xacro')
    urdf_path_fp3 = os.path.join(franka_description_path, 'urdfs', 'fp3_franka_hand.urdf')
    
    # Set IGN_GAZEBO_RESOURCE_PATH to include the model path
    model_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(franka_description_path),
            os.path.join(franka_description_path, 'meshes', 'robot_arms', 'fp3', 'collision'),
            ':',
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
        ]
    )

    # Set the model path in GAZEBO_MODEL_PATH as well
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            os.path.join(franka_description_path),
            ':',
            os.environ.get('GAZEBO_MODEL_PATH', '')
        ]
    )

    # Include the Gazebo empty world launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    )

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_path_fp3).read(),
            'use_sim_time': True
        }]
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'fp3_franka_hand',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ]
    )

    return LaunchDescription([
        model_path,
        gazebo_model_path,
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
    ])