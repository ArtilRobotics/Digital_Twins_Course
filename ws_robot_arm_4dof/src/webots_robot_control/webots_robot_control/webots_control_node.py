import rclpy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy


# Límites para cada motor (debes ajustar estos valores según las especificaciones de tu robot)

M2_MIN = -0.4363323129985824
M2_MAX = 3.5779249665883754

M3_MIN = -2.007128639793479
M3_MAX = 2.007128639793479

M4_MIN = -2.007128639793479
M4_MAX = 2.007128639793479

M5_MIN = -1.40943951023931953
M5_MAX = 0.27453292519943295


class MyRobotDriver:
    def init(self, webots_node, properties):
        self._robot = webots_node.robot

        # Inicializamos los motores
        self._m1_motor = self._robot.getDevice('m1_continuous')
        self._m2_motor = self._robot.getDevice('m2')
        self._m3_motor = self._robot.getDevice('m3')
        self._m4_motor = self._robot.getDevice('m4')
        self._m5_motor = self._robot.getDevice('m5')


        self._sensor_1 = self._robot.getDevice("m1_continuous_sensor")
        self._sensor_2 = self._robot.getDevice("m2_sensor")
        self._sensor_3 = self._robot.getDevice("m3_sensor")
        self._sensor_4 = self._robot.getDevice("m4_sensor")
        self._sensor_5 = self._robot.getDevice("m5_sensor")

        for sensor in [self._sensor_1, self._sensor_2, self._sensor_3, self._sensor_4, self._sensor_5]:
            sensor.enable(10)

        for motor in [self._m1_motor, self._m2_motor, self._m3_motor, self._m4_motor, self._m5_motor]:
            motor.setPosition(float('inf')) 
            motor.setVelocity(0.0)

        # Inicializamos los targets para cada motor
        self.__target_m1 = 0.0
        self.__target_m2 = 0.0
        self.__target_m3 = 0.0
        self.__target_m4 = 0.0
        self.__target_m5 = 0.0

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')

        # Crear suscripciones para cada motor
        self.__node.create_subscription(Float32, 'cmd_m1', self.__cmd_m1_callback, 1)
        self.__node.create_subscription(Float32, 'cmd_m2', self.__cmd_m2_callback, 1)
        self.__node.create_subscription(Float32, 'cmd_m3', self.__cmd_m3_callback, 1)
        self.__node.create_subscription(Float32, 'cmd_m4', self.__cmd_m4_callback, 1)
        self.__node.create_subscription(Float32, 'cmd_m5', self.__cmd_m5_callback, 1)
        # Crear un publisher para los estados de los sensores
        self._state_pub = self.__node.create_publisher(Joy, '/states', 10)

    # Callbacks para cada motor
    def __cmd_m1_callback(self, float32_msg):
        self.__target_m1 = float32_msg.data
        #print(f"Recibido M1: {float32_msg.data}")

    def __cmd_m2_callback(self, float32_msg):
        self.__target_m2 = float32_msg.data
        #print(f"Recibido M2: {float32_msg.data}")

    def __cmd_m3_callback(self, float32_msg):
        self.__target_m3 = float32_msg.data
        #print(f"Recibido M3: {float32_msg.data}")

    def __cmd_m4_callback(self, float32_msg):
        self.__target_m4 = float32_msg.data
        #print(f"Recibido M4: {float32_msg.data}")

    def __cmd_m5_callback(self, float32_msg):
        self.__target_m5 = float32_msg.data
        #print(f"Recibido M5: {float32_msg.data}")

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # Obtener las posiciones actuales de los motores
        current_m1 = self._sensor_1.getValue()
        current_m2 = self._sensor_2.getValue()
        current_m3 = self._sensor_3.getValue()
        current_m4 = self._sensor_4.getValue()
        current_m5 = self._sensor_5.getValue()

        # Limitar las velocidades dependiendo de la dirección del movimiento

        # Motor 1: permitir moverse si se aleja del límite
        self._m1_motor.setVelocity(self.__target_m1)

        # Motor 2
        if current_m2 <= M2_MIN and self.__target_m2 < 0.0:
            self._m2_motor.setVelocity(0.0)
        elif current_m2 >= M2_MAX and self.__target_m2 > 0.0:
            self._m2_motor.setVelocity(0.0)
        else:
            self._m2_motor.setVelocity(self.__target_m2)

        # Motor 3
        if current_m3 <= M3_MIN and self.__target_m3 < 0.0:
            self._m3_motor.setVelocity(0.0)
        elif current_m3 >= M3_MAX and self.__target_m3 > 0.0:
            self._m3_motor.setVelocity(0.0)
        else:
            self._m3_motor.setVelocity(self.__target_m3)

        # Motor 4
        if current_m4 <= M4_MIN and self.__target_m4 < 0.0:
            self._m4_motor.setVelocity(0.0)
        elif current_m4 >= M4_MAX and self.__target_m4 > 0.0:
            self._m4_motor.setVelocity(0.0)
        else:
            self._m4_motor.setVelocity(self.__target_m4)

        # Motor 5
        if current_m5 <= M5_MIN and self.__target_m5 < 0.0:
            self._m5_motor.setVelocity(0.0)
        elif current_m5 >= M5_MAX and self.__target_m5 > 0.0:
            self._m5_motor.setVelocity(0.0)
        else:
            self._m5_motor.setVelocity(self.__target_m5)

        # Publicar los estados del brazo
        q = [
            current_m1, 
            current_m2, 
            current_m3, 
            current_m4, 
            0.0, 0.0, 0.0, 0.0
        ]
        self.send_states(q)




    def send_states(self, u):
        # Crear el mensaje de estados
        state_msg = Joy()
        state_msg.header.frame_id = "base_link"
        state_msg.header.stamp = self.__node.get_clock().now().to_msg()
        state_msg.axes = u
        # Publicar los valores de los sensores
        self._state_pub.publish(state_msg)
