from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    root_path = os.path.join(os.getcwd(), 'src', 'teste-robot')

    pkg_path = get_package_share_directory('teste_bot')
    xacro_path = os.path.join(root_path, 'description', 'robot.urdf.xacro')
    urdf_path = os.path.join(root_path, 'description', 'robot.urdf')
    
    package_name = 'teste_bot'

    # Definição do argumento de ativação do teleop
    declare_teleop_arg = DeclareLaunchArgument(
        'teleop',
        default_value='false',
        description='Flag to activate teleop'
    )


    # Caminho para a pasta de configuração do bridge ROS2-Gazebo
    config_file = os.path.join(root_path, 'config', 'bridge.yaml')

    # Executa o comando xacro para gerar o arquivo URDF
    xacro_to_urdf = ExecuteProcess(
        cmd=['xacro', xacro_path, '-o', urdf_path],

    )

    # Executa o comando para iniciar o Robot State Publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Declare the launch argument for gazebo
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', 'empty.sdf'],
        output='screen'
    )

    # Executa o Nó para ponte ROS2-Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': config_file}],
    )

    # Executa Spawn do robô no Gazebo
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_spawn_model.launch.py'
            ])
        ]),
        launch_arguments={
            'file' : urdf_path,
            'entity_name' : 'robot',
            'topic' : '/robot_description',
            'z' : '0.05'
        }.items()
    )

    # Executa o nó de teleop
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='gnome-terminal -- bash -c',
        shell=True,
        condition=IfCondition(LaunchConfiguration('teleop')), 
        parameters=[{
            'speed': 0.5,
            'turn': 1.0
        }]
    )

    return LaunchDescription(
        [declare_teleop_arg,
         xacro_to_urdf,
         rsp,
         gazebo_process,
         bridge,
         spawn,
         teleop_node]
    )
# generate_launch_description()