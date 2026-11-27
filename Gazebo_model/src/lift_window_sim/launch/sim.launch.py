"""Starter Gazebo med fasaden, legger inn vindusvaskeroboten og starter kontrollerne.
Kjøres med: ros2 launch lift_window_sim sim.launch.py
"""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = 'lift_window_sim'
    pkg_path = get_package_share_directory(package_name)

    # Robotmodellen (Xacro -> URDF)
    xacro_file = os.path.join(pkg_path, 'urdf', 'lift_arm.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # Verdenen med fasaden og 40 N-kraften på dysen
    world_file = os.path.join(pkg_path, 'worlds', 'facade_load.sdf')

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + world_file,
                          'on_exit_shutdown': 'true'}.items())

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str),
                     'use_sim_time': True}])

    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'lift_window', '-topic', 'robot_description', '-allow_renaming', 'false'])

    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    node_joint_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '120'])

    node_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['arm_controller', '--controller-manager-timeout', '120'])

    # Sjekker at teleskoptrinnene følger uttrekket før testene startes
    node_check_chain = Node(
        package=package_name,
        executable='check_chain',
        output='screen',
        parameters=[{'use_sim_time': True}])

    # Kontrollerne startes først når roboten er opprettet i Gazebo.
    # Jeg brukte KI til å forstå OnProcessExit og fikk forslag til hvordan
    # jeg kunne starte kontrollerne etter at roboten var opprettet i Gazebo.
    # Jeg fikk også hjelp til å feilsøke oppstarten og kontrolleroppsettet.
    def after_spawn(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='Roboten kunne ikke opprettes. Sjekk Gazebo-terminalen.'), EmitEvent(event=Shutdown())]
        return [node_joint_broadcaster]

    def after_broadcaster(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='Kontrollerbehandleren svarer ikke. Sjekk utskriften fra gz_ros2_control.'), EmitEvent(event=Shutdown())]
        return [node_arm_controller]

    def after_arm_controller(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='arm_controller kunne ikke aktiveres.'), EmitEvent(event=Shutdown())]
        return [node_check_chain]

    def after_check(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='CHAIN NOT READY: kjør gaa_til_start fra MATLAB før kjor_test.')]
        return [LogInfo(msg='TOP READY: Kjør kjor_test("last"), kjor_test("top") eller kjor_test("vinkel") fra MATLAB.')]

    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(target_action=node_spawn_entity, on_exit=after_spawn)),
        RegisterEventHandler(OnProcessExit(target_action=node_joint_broadcaster, on_exit=after_broadcaster)),
        RegisterEventHandler(OnProcessExit(target_action=node_arm_controller, on_exit=after_arm_controller)),
        RegisterEventHandler(OnProcessExit(target_action=node_check_chain, on_exit=after_check)),
        gazebo_sim,
        node_robot_state_publisher,
        node_ros_gz_bridge,
        node_spawn_entity,
    ])
