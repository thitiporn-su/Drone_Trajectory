from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os, yaml
import xacro

def generate_launch_description():
    pkg = get_package_share_directory('uav_description')
    path = os.path.join(pkg,'robot','robot.xacro')
    robot_desc = xacro.process_file(path).toxml()

    # robot_desc = 0
    
    rviz_path = os.path.join(pkg,'config','_display.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen'
    )
    
    robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output='screen',
        parameters = [{'robot_description':robot_desc,'use_sim_time': True}]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )
    
    robot_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'drone',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            ]
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"]
    )

    # velocity_controllers = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["velocity_controllers", "--controller-manager", "controller_manager"]
    # )

    # motor_velocity_update = Node(
    #     package="uav_description",
    #     executable="sb_motor_controller.py"
    # )
   
    
    launch_description = LaunchDescription()
    # launch_description.add_action(rviz)
    # launch_description.add_action(declare_world_fname)
    launch_description.add_action(gazebo)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(robot_spawner)
    launch_description.add_action(joint_state_broadcaster)
    # launch_description.add_action(velocity_controllers)
    # launch_description.add_action(motor_velocity_update)
    # launch_description.add_action(joint_state_publisher_gui)
    return launch_description