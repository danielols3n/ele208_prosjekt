"""Starter Gazebo og ROS 2-kontrollerne med robotmodellen fra Xacro.
"""
from pathlib import Path
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
    share=Path(get_package_share_directory('lift_window_sim'))
    gz_share=Path(get_package_share_directory('ros_gz_sim'))
    description=xacro.process_file(str(share/'urdf/lift_arm.urdf.xacro'),
        mappings={'controllers_file':str(share/'config/controllers.yaml')}).toxml()
    gazebo=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz_share/'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args':'-r '+str(share/'worlds/facade_load.sdf'),'on_exit_shutdown':'true'}.items())
    state=Node(package='robot_state_publisher',executable='robot_state_publisher',output='screen',
        parameters=[{'robot_description':ParameterValue(description,value_type=str),'use_sim_time':True}])
    clock=Node(package='ros_gz_bridge',executable='parameter_bridge',output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    spawn=Node(package='ros_gz_sim',executable='create',output='screen',
        arguments=['-name','lift_window','-topic','robot_description','-allow_renaming','false'])
    jsb=Node(package='controller_manager',executable='spawner',output='screen',
        arguments=['joint_state_broadcaster','--controller-manager-timeout','120'])
    controller=Node(package='controller_manager',executable='spawner',output='screen',
        arguments=['joint_trajectory_controller','--controller-manager-timeout','120'])
    check=Node(package='lift_window_sim',executable='check_chain',output='screen',
        parameters=[{'use_sim_time':True}])
    def after_spawn(event,context):
        # Kontrollerne startes først når roboten er opprettet i Gazebo.
        if event.returncode!=0:
            return [LogInfo(msg='Roboten kunne ikke opprettes. Sjekk Gazebo-terminalen.'),EmitEvent(event=Shutdown())]
        return [jsb]
    def after_jsb(event,context):
        if event.returncode!=0:
            return [LogInfo(msg='Kontrollerbehandleren svarer ikke. Sjekk utskriften fra gz_ros2_control.'),EmitEvent(event=Shutdown())]
        return [controller]
    def after_controller(event,context):
        if event.returncode!=0:
            return [LogInfo(msg='Banekontrolleren kunne ikke aktiveres.'),EmitEvent(event=Shutdown())]
        return [check]
    def after_check(event,context):
        if event.returncode!=0:
            return [LogInfo(msg='Teleskopleddene fulgte ikke sammen. Sjekk utskriften fra check_chain.'),EmitEvent(event=Shutdown())]
        return [LogInfo(msg='TOP READY: Kjør kjor_test("last"), kjor_test("top") eller kjor_test("vinkel") fra MATLAB.')]
    # Jeg brukte KI til å forstå OnProcessExit og fikk forslag til hvordan
    # jeg kunne starte kontrollerne etter at roboten var opprettet i Gazebo.
    # Jeg fikk også hjelp til å feilsøke oppstarten og kontrolleroppsettet.
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=after_spawn)),
        RegisterEventHandler(OnProcessExit(target_action=jsb,on_exit=after_jsb)),
        RegisterEventHandler(OnProcessExit(target_action=controller,on_exit=after_controller)),
        RegisterEventHandler(OnProcessExit(target_action=check,on_exit=after_check)),
        gazebo,state,clock,spawn])
