import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.events import matches_action
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    pkg_minibot = get_package_share_directory('minibot')
    default_slam_params = os.path.join(pkg_minibot, 'config', 'minibot_slam_mapping.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file', default_value=default_slam_params,
        description='Path to params file')

    # Nodo SLAM Toolbox (Lifecycle)
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ])

    # 1. Evento de Configuración: Se lanza INMEDIATAMENTE al iniciar el nodo
    configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_toolbox_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ]
        )
    )

    # 2. Evento de Activación: Se lanza con RETRASO de 2 segundos
    # Esto da tiempo a que termine la configuración y evita el error "Transition not registered"
    activate_event = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_toolbox_node,
            on_start=[
                TimerAction(
                    period=2.0, # Esperamos 2 segundos
                    actions=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(slam_toolbox_node),
                                transition_id=Transition.TRANSITION_ACTIVATE,
                            )
                        )
                    ]
                )
            ]
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params,
        slam_toolbox_node,
        configure_event,
        activate_event
    ])