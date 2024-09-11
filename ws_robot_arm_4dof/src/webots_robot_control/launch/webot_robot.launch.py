from launch import LaunchDescription
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from ament_index_python.packages import get_package_share_directory
import os
import launch 
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Obtiene la ruta del paquete 'webots_robot_control'
    package_dir = get_package_share_directory('webots_robot_control')

    # Construye la ruta completa hacia el archivo .wbt dentro de 'worlds'
    world_path = os.path.join(package_dir, 'mini4DoF', 'worlds', 'mini4DoF.wbt')

    py_path = os.path.join(package_dir, 'resource', 'control_nodes.py')

    # Lanza Webots con el archivo del mundo especificado
    webots = WebotsLauncher(world=world_path)

    # Rutas a URDF y parámetros de control
    path_urdf = os.path.join(package_dir, 'mini4DoF', 'protos', 'urdf', 'robot_control.urdf')

    # Controlador Webots
    my_robot_driver = WebotsController(
        robot_name='robot',
        parameters=[
            {'robot_description': path_urdf},
        ]
    )

    script = ExecuteProcess(
            cmd=['python3', py_path],
            name='virtual_joystick',
            output='screen'
        )
    
    # Conexión al controlador
    wait_for_connection = WaitForControllerConnection(
        target_driver=my_robot_driver,
        nodes_to_start=[]
    )
    
    node_control = Node(
            package='webots_robot_control',
            executable='webots_control_node',
        )

    # Retorna la descripción del lanzamiento
    return LaunchDescription([
        webots,
        my_robot_driver,
        node_control,
        script,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])

