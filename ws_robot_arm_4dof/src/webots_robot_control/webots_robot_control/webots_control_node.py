import rclpy
from std_msgs.msg import Float32

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class MyRobotDriver:
    def init(self, webots_node, properties):
        self._robot = webots_node.robot

        # Inicializamos los motores
        self._m1_motor = self._robot.getDevice('m1_continuous')
        self._m2_motor = self._robot.getDevice('m2')
        self._m3_motor = self._robot.getDevice('m3')
        self._m4_motor = self._robot.getDevice('m4')
        self._m5_motor = self._robot.getDevice('m5')


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

    # Callbacks para cada motor
    def __cmd_m1_callback(self, float32_msg):
        self.__target_m1 = float32_msg.data
        print(f"Recibido M1: {float32_msg.data}")

    def __cmd_m2_callback(self, float32_msg):
        self.__target_m2 = float32_msg.data
        print(f"Recibido M2: {float32_msg.data}")

    def __cmd_m3_callback(self, float32_msg):
        self.__target_m3 = float32_msg.data
        print(f"Recibido M3: {float32_msg.data}")

    def __cmd_m4_callback(self, float32_msg):
        self.__target_m4 = float32_msg.data
        print(f"Recibido M4: {float32_msg.data}")

    def __cmd_m5_callback(self, float32_msg):
        self.__target_m5 = float32_msg.data
        print(f"Recibido M5: {float32_msg.data}")

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # Establecer las velocidades para cada motor
        self._m1_motor.setVelocity(self.__target_m1)
        self._m2_motor.setVelocity(self.__target_m2)
        self._m3_motor.setVelocity(self.__target_m3)
        self._m4_motor.setVelocity(self.__target_m4)
        self._m5_motor.setVelocity(self.__target_m5)
        
