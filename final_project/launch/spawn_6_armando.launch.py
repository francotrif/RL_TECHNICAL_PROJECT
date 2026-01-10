import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def spawn_armando(robot_id, x_pose, y_pose, yaw):
    """
    Spawna un singolo Armando con namespace e configurazione unici
    
    STRATEGIA: OnProcessExit invece di TimerAction!
    - JSB spawner parte
    - DOPO che JSB finisce → position_controller spawner parte
    - DOPO che position finisce → gripper_controller spawner parte
    
    Questo garantisce che YAML sia caricato prima di spawn controller e tutti i controllori partino con successo.
    
    Args:
        robot_id: ID del robot (1-6)
        x_pose, y_pose: Posizione nel mondo
        yaw: Orientamento (radianti)
    
    Returns:
        Lista di nodi ROS2 per questo robot
    """
    
    namespace = f'armando_{robot_id}'
    name = f'armando_{robot_id}'
    
    # ✅ Path assoluti per evitare problemi PathJoinSubstitution
    pkg_share_path = get_package_share_directory('final_project')
    
    yaml_file = f'armando/armando_controllers_{robot_id}.yaml'
    yaml_absolute_path = os.path.join(pkg_share_path, 'config', yaml_file)
    
    urdf_absolute_path = os.path.join(pkg_share_path, 'urdf', 'armando', 'arm.urdf.xacro')
    
    mesh_dir = os.path.join(pkg_share_path, 'meshes', 'armando')
    
    # Robot description con path assoluti
    robot_description_content = Command([
        'xacro ',
        urdf_absolute_path,
        ' namespace:=', namespace,
        ' yaml_file:=', yaml_absolute_path,
        ' mesh_dir:=', mesh_dir
    ])
    
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', name,
            '-topic', f'/{namespace}/robot_description',
            '-x', str(x_pose),
            '-y', str(y_pose),
            '-z', '0.0',
            '-Y', str(yaw)
        ],
        output='screen'
    )

    # Joint state broadcaster
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c', f'/{namespace}/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )
    
    # Position controller spawner
    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'position_controller',
            '-c', f'/{namespace}/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )
    
    # Gripper controller spawner
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '-c', f'/{namespace}/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )
    
    # ✅ CHIAVE: RegisterEventHandler con OnProcessExit!
    # Position controller spawna DOPO che JSB finisce
    position_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[position_controller_spawner]
        )
    )
    
    # Gripper controller spawna DOPO che position finisce
    gripper_after_position = RegisterEventHandler(
        OnProcessExit(
            target_action=position_controller_spawner,
            on_exit=[gripper_controller_spawner]
        )
    )
    
    return [
        robot_state_publisher,
        spawn_entity,
        joint_state_spawner,
        position_after_jsb,      # Event handler invece di nodo diretto!
        gripper_after_position   # Event handler invece di nodo diretto!
    ]


def generate_launch_description():
    """
    Spawna 6 Armando robots nel mondo Fra2mo airport
    
    STRATEGIA:
    - Path assoluti (no PathJoinSubstitution)
    - OnProcessExit per spawn controller sequenziale
    - Delay tra robot per evitare sovraccarico Gazebo
    
    TIMING:
    Robot N: spawna a T = (N-1) * 5 secondi
    Per ogni robot:
      - JSB spawner parte
      - Position spawner parte DOPO che JSB finisce
      - Gripper spawner parte DOPO che position finisce
    
    TOTALE: ~35 secondi per spawn completo di tutti e 6
    """
    
    # Posizioni Armando 
    armando_positions = {
        1: {'x': -3.5, 'y': -7.5, 'yaw':  0.0},  # Aereo 1 - Colonna DX
        2: {'x': -3.5, 'y':  7.5, 'yaw': 3.14},  # Aereo 2 - Colonna SX
        3: {'x':  2.5, 'y': -7.5, 'yaw':  0.0},  # Aereo 3 - Colonna DX
        4: {'x':  2.5, 'y':  7.5, 'yaw': 3.14},  # Aereo 4 - Colonna SX        
        5: {'x':  8.5, 'y': -7.5, 'yaw':  0.0},  # Aereo 5 - Colonna DX
        6: {'x':  8.5, 'y':  7.5, 'yaw': 3.14},  # Aereo 6 - Colonna SX
    } 
    
    all_nodes = []
    
    # Spawn tutti e 6 Armando con delay tra robot
    for robot_id in range(1, 7):
        pos = armando_positions[robot_id]
        robot_nodes = spawn_armando(
            robot_id=robot_id,
            x_pose=pos['x'],
            y_pose=pos['y'],
            yaw=pos['yaw']
        )
        
        if robot_id == 1:
            # Primo robot: spawn immediato
            all_nodes.extend(robot_nodes)
        else:
            # Altri robot: delay di 5 secondi tra ciascuno
            delay_seconds = (robot_id - 1) * 5.0
            
            # Wrap tutti i nodi in TimerAction
            delayed_nodes = TimerAction(
                period=delay_seconds,
                actions=robot_nodes
            )
            all_nodes.append(delayed_nodes)
    
    return LaunchDescription(all_nodes)