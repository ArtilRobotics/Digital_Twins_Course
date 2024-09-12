import pygame
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

# Inicializar Pygame
pygame.init()

# Configuración de pantalla
screen = pygame.display.set_mode((300, 300))
pygame.display.set_caption("Sliders Virtuales")

# Definir colores
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GRAY = (200, 200, 200)

# Configuración de sliders
slider_width = 200
slider_height = 20
slider_x = 50
slider_y_start = 50
slider_spacing = 50

# Datos de los sliders
sliders = [
    {"label": "joint1", "value": 0.0, "min": -math.pi, "max": math.pi, "rect": pygame.Rect(slider_x, slider_y_start, slider_width, slider_height)},
    {"label": "joint2", "value": 0.0, "min": -math.pi, "max": math.pi, "rect": pygame.Rect(slider_x, slider_y_start + slider_spacing, slider_width, slider_height)},
    {"label": "joint3", "value": 0.0, "min": -math.pi, "max": math.pi, "rect": pygame.Rect(slider_x, slider_y_start + 2 * slider_spacing, slider_width, slider_height)},
    {"label": "joint4", "value": 0.0, "min": -math.pi, "max": math.pi, "rect": pygame.Rect(slider_x, slider_y_start + 3 * slider_spacing, slider_width, slider_height)},
    {"label": "joint5", "value": 0.0, "min": -math.pi, "max": math.pi, "rect": pygame.Rect(slider_x, slider_y_start + 4 * slider_spacing, slider_width, slider_height)}
]

# Variables de estado para arrastrar sliders
dragging = [False] * len(sliders)

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

    def publish_values(self, values):
        # Publicar valores en los tópicos correspondientes
        self.publisher_m1.publish(Float32(data=values[0]))
        self.publisher_m2.publish(Float32(data=values[1]))
        self.publisher_m3.publish(Float32(data=values[2]))
        self.publisher_m4.publish(Float32(data=values[3]))
        self.publisher_m5.publish(Float32(data=values[4]))

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
            for i, slider in enumerate(sliders):
                if slider["rect"].collidepoint(event.pos):
                    dragging[i] = True

        if event.type == pygame.MOUSEBUTTONUP:
            dragging = [False] * len(sliders)

        if event.type == pygame.MOUSEMOTION:
            for i, slider in enumerate(sliders):
                if dragging[i]:
                    mouse_x = event.pos[0]
                    knob_x = min(max(mouse_x, slider["rect"].left), slider["rect"].right)
                    # Calcular el valor del slider basado en la posición del knob
                    slider["value"] = ((knob_x - slider["rect"].left) / slider_width) * (slider["max"] - slider["min"]) + slider["min"]

    # Dibujar sliders
    values = []
    for slider in sliders:
        # Dibujar la barra del slider
        pygame.draw.rect(screen, GRAY, slider["rect"])
        # Dibujar el knob
        knob_x = int((slider["value"] - slider["min"]) / (slider["max"] - slider["min"]) * slider_width) + slider["rect"].left
        pygame.draw.circle(screen, RED, (knob_x, slider["rect"].centery), 10)
        # Dibujar la etiqueta y el valor actual
        font = pygame.font.SysFont(None, 24)
        label_surface = font.render(f"{slider['label']}: {slider['value']:.2f}", True, BLACK)
        screen.blit(label_surface, (slider["rect"].left, slider["rect"].top - 30))
        values.append(slider["value"])

    # Publicar valores en ROS 2
    node.publish_values(values)

    # Actualizar pantalla
    pygame.display.flip()
