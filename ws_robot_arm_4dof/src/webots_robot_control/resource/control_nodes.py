import pygame
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Inicializar Pygame
pygame.init()

# Configuración de pantalla
screen = pygame.display.set_mode((150, 430))
pygame.display.set_caption("Joysticks Virtuales")

# Definir colores
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)

# Posiciones y tamaños de los joysticks virtuales
joystick_radius = 33         # Radio del área del joystick (1/3 del tamaño original)
joystick_knob_radius = 7     # Radio del botón del joystick (1/3 del tamaño original)
joystick_x = 50             # Coordenada X para todos los joysticks
joystick_y_start = 50       # Coordenada Y inicial para el primer joystick
joystick_y_spacing = 80     # Espacio vertical entre los joysticks

# Datos de los joysticks
joystick_data = [
    {"label": "joint1", "center": (joystick_x, joystick_y_start), "knob_x": joystick_x, "knob_y": joystick_y_start},
    {"label": "joint2", "center": (joystick_x, joystick_y_start + joystick_y_spacing), "knob_x": joystick_x, "knob_y": joystick_y_start + joystick_y_spacing},
    {"label": "joint3", "center": (joystick_x, joystick_y_start + 2 * joystick_y_spacing), "knob_x": joystick_x, "knob_y": joystick_y_start + 2 * joystick_y_spacing},
    {"label": "joint4", "center": (joystick_x, joystick_y_start + 3 * joystick_y_spacing), "knob_x": joystick_x, "knob_y": joystick_y_start + 3 * joystick_y_spacing},
    {"label": "joint5", "center": (joystick_x, joystick_y_start + 4 * joystick_y_spacing), "knob_x": joystick_x, "knob_y": joystick_y_start + 4 * joystick_y_spacing}
]

# Estado de los joysticks
dragging = [False] * len(joystick_data)

# Función para limitar el movimiento del knob del joystick
def limit_joystick(knob_x, center_x, radius):
    if knob_x < center_x - radius:
        knob_x = center_x - radius
    elif knob_x > center_x + radius:
        knob_x = center_x + radius
    return knob_x

# Clase de nodo para ROS 2
class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        # Crear publicadores individualmente sin listas
        self.publisher_m1 = self.create_publisher(Float32, '/cmd_m1', 10)
        self.publisher_m2 = self.create_publisher(Float32, '/cmd_m2', 10)
        self.publisher_m3 = self.create_publisher(Float32, '/cmd_m3', 10)
        self.publisher_m4 = self.create_publisher(Float32, '/cmd_m4', 10)
        self.publisher_m5 = self.create_publisher(Float32, '/cmd_m5', 10)

    def publish_values(self, speed1, speed2, speed3, speed4, speed5):
        # Publicar valores en los tópicos correspondientes
        self.publisher_m1.publish(Float32(data=speed1))
        self.publisher_m2.publish(Float32(data=speed2))
        self.publisher_m3.publish(Float32(data=speed3))
        self.publisher_m4.publish(Float32(data=speed4))
        self.publisher_m5.publish(Float32(data=speed5))
        self.get_logger().info(f'Publicando: {speed1}, {speed2}, {speed3}, {speed4}, {speed5}')

# Inicializar ROS 2
rclpy.init()
node = JoystickPublisher()

# Bucle principal
running = True
while running:
    screen.fill(WHITE)

    # Detectar eventos
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            rclpy.shutdown()
            sys.exit()

        if event.type == pygame.MOUSEBUTTONDOWN:
            # Detectar si se ha hecho clic en algún botón del joystick
            for i, data in enumerate(joystick_data):
                mouse_x, mouse_y = event.pos
                distance = ((mouse_x - data["knob_x"])**2 + (mouse_y - data["knob_y"])**2)**0.5
                if distance <= joystick_knob_radius:
                    dragging[i] = True

        if event.type == pygame.MOUSEBUTTONUP:
            # Soltar el botón del joystick
            for i in range(len(dragging)):
                dragging[i] = False
                joystick_data[i]["knob_x"] = joystick_data[i]["center"][0]  # Volver el knob al centro

        if event.type == pygame.MOUSEMOTION:
            # Mover el knob solo en el eje X
            for i, data in enumerate(joystick_data):
                if dragging[i]:
                    mouse_x, _ = event.pos
                    data["knob_x"] = limit_joystick(mouse_x, data["center"][0], joystick_radius)

    # Dibujar los joysticks
    speeds = []
    for data in joystick_data:
        # Área del joystick
        pygame.draw.circle(screen, BLACK, data["center"], joystick_radius, 3)
        # Botón del joystick
        pygame.draw.circle(screen, RED, (int(data["knob_x"]), int(data["knob_y"])), joystick_knob_radius)
        # Etiqueta del joystick a la derecha
        font = pygame.font.SysFont(None, 24)
        label_surface = font.render(data["label"], True, BLACK)
        screen.blit(label_surface, (data["center"][0] + joystick_radius + 10, data["center"][1] - label_surface.get_height() // 2))

        # Calcular la velocidad basada en la posición del knob
        knob_position = data["knob_x"] - data["center"][0]
        speed = (knob_position / joystick_radius) * 0.2  # Escalar la posición del knob para obtener velocidad
        speeds.append(speed)

    # Publicar velocidades en ROS 2
    node.publish_values(speeds[0], speeds[1], speeds[2], speeds[3], speeds[4])

    # Actualizar pantalla
    pygame.display.flip()
