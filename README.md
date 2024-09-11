# Ejecución de Launch Files

## 1. ws_robot_arm4dof

Para ejecutar el launch de `ws_robot_arm4dof`, usa el siguiente comando:


`ros2 launch webots_robot_control webot_robot.launch.py`

Al ejecutarlo, se abrirá una simulación donde podrás mover cada joint del robot. A continuación, se muestra una imagen del robot en movimiento:
![Screenshot from 2024-09-10 21-29-56](https://github.com/user-attachments/assets/7334446e-f63f-4324-a458-3ddaed18ce8e)

## 2.ws_test_webots
Para ejecutar el robot en `ws_test_webots`, usa el siguiente comando:
`ros2 launch my_package robot_launch.py`
Este comando ejecuta un robot que se mueve de forma autónoma gracias a un paquete ROS2 y evita colisiones utilizando sus sensores.

## 3. ws_webots_epuck
En este proyecto más completo, puedes ejecutar varios launch. Para iniciar el proyecto, utiliza el siguiente comando:
`ros2 launch epuck_ros2 robot_launch.py`
Puedes mover el robot publicando en el tópico /cmd_vel de la siguiente manera:
`ros2 topic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
`
Además, puedes utilizar nav2 y otras herramientas lanzando el siguiente comando:
`ros2 launch webots_ros2_epuck robot_launch.py rviz:=true nav:=true`
Más información sobre este proyecto en: https://github.com/cyberbotics/webots_ros2/wiki/Tutorial-E-puck-for-ROS2-Beginners 
