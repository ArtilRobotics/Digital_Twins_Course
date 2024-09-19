# ws_webots_epuck
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

![WhatsApp Image 2024-09-19 at 13 38 02_c871da24](https://github.com/user-attachments/assets/a733f383-0cb9-49a0-8fdf-8b22ebe69cd3)
