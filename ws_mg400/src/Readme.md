Para lanzar una simulacion del MG400 en rviz2 se puede utilizar:
```ros2 launch mg400_bringup display.launch.py```

Y una vez se tenga conectado al MG400 mediante ethernet se puede lanzar:
```ros2 launch mg400_bringup main.launch.py ```

El robot configurado tiene la ip ```192.168.1.6```, si se tuviera que cambiar, hay que hacerlo en el launch: ```main.launch.py``` puesto que ya esta definida como una variable 

![image](https://github.com/user-attachments/assets/9eb57e35-5719-45b9-b599-4b22a21b179d)
