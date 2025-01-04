from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory("sbr_locomotion")   # Enter your package name here
    urdf_file = os.path.join(pkg_path, 'urdf', 'sbr.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'custom.world')

    robot_description = Command(['xacro ', urdf_file])
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    params={'robot_description':doc.toxml()}


    robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[params]
    )

    # Launch Gazebo and spawn the robot
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_file}.items()
    )

    # Spawn Entity
    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'robot', '-x', '0', '-y', '0', '-z', '0'],  # Adjust the z value
        output='screen'
    )

    # IMU Node
    imu = Node(
        package='sbr_locomotion',
        executable='imuReader.py',
        output='screen')          

    # LQR Node 
    lqr = Node(
        package='sbr_locomotion',
        executable='lqr.py',      
        output='screen')          
    
    # Differential Drive Node 
    differentialDrive = Node(
        package='sbr_locomotion',
        executable='differentialDrive.py',
        output='screen')            
    
    return LaunchDescription([
        DeclareLaunchArgument(name='headless', default_value='true', description='Set to "false" to run headless.'),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        imu,
        lqr,
        differentialDrive,
    ])