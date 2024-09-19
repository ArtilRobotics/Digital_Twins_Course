Para instalar es necesario ejecutar de la carpeta scripts los siguientes archivos respetando el orden:
- build.sh
- gazebo.sh
- opencv.sh
- orbslam.sh
- install.sh
Una vez que se haya compilado el paquete, en este caso ```install.sh``` hace el proceso mediante ```colcon build```, por lo que este último ya no es necesario

Para lanzar el paquete se necesita ejecutar ```run.sh``` sin embargo se puede hacer mediante:

```cd ../workspace```
```. install/setup.bash```
```cd src```
```ros2 launch launch.py```
Lo que lanzara todos los paquetes de Tello, cuidado al manipular el dron, dado que el paquete no es de confianza 💀
