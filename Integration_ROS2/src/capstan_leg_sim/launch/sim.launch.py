import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'capstan_leg_sim'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # Генерация URDF из Xacro
    xacro_file = os.path.join(pkg_dir, 'urdf', 'leg.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}
    
    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Запуск Gazebo Classic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # Спавн модели в Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'capstan_leg', '-z', '0.5'],
        output='screen'
    )

    # Загрузчики контроллеров ros2_control
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'position_controller'],
        output='screen'
    )

    # Нода инверсной кинематики (ESP32 аналог)
    leg_controller_node = Node(
        package=pkg_name,
        executable='leg_controller_node',
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_position_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_position_controller,
                on_exit=[leg_controller_node],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])