import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # Имя вашего ROS-пакета
    pkg_name = 'capstan_leg_sim'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Путь к файлу описания робота. 
    # ВНИМАНИЕ: Если ваш файл называется не 'robot.urdf.xacro', 
    # замените имя в конце этой строки на актуальное (например, 'leg.urdf').
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro') 
    
    # 1. Запуск robot_state_publisher (собирает URDF и публикует в топик /robot_description)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': True
        }]
    )

    # 2. Запуск Gazebo Harmonic (с пустым миром, флаг -r запускает симуляцию сразу)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. Добавление (спавн) модели робота из топика ROS 2 в мир Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'capstan_robot_dog',
            '-z', '0.5' # Высота появления (0.5 метра, чтобы робот не провалился в пол)
        ],
        output='screen'
    )

    # 4. Мост (Bridge) для синхронизации времени и топиков между Gazebo и ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Синхронизация времени
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Проброс топика IMU из Gazebo в ROS 2
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        bridge
    ])
