import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
import numpy as np
from scipy.io import savemat
import os

# Características del Brazo
l = [0.0676, 0.06883, 0.06883, 0.15916]

# Variables globales de Odometría del Drone
q1 = 1.0
q2 = 0
q3 = 0.0
q4 = 0.0
q1_p = 0
q2_p = 0
q3_p = 0.0
q4_p = 0.0

def CDArm4DOF(l, q):
    q1, q2, q3, q4 = q
    hx = np.cos(q1) * (l[2] * np.cos(q2 + q3) + l[1] * np.cos(q2) + l[3] * np.cos(q2 + q3 + q4))
    hy = np.sin(q1) * (l[2] * np.cos(q2 + q3) + l[1] * np.cos(q2) + l[3] * np.cos(q2 + q3 + q4))
    hz = l[0] + l[2] * np.sin(q2 + q3) + l[1] * np.sin(q2) + l[3] * np.sin(q2 + q3 + q4)
    return np.array([hx, hy, hz])

def jacobiana_Brazo4DOF(L, q):
    l1, l2, l3, l4 = L
    q1, q2, q3, q4 = q
    J = np.array([
        [-np.sin(q1) * (l3 * np.cos(q2 + q3) + l2 * np.cos(q2) + l4 * np.cos(q2 + q3 + q4)),
         -np.cos(q1) * (l3 * np.sin(q2 + q3) + l2 * np.sin(q2) + l4 * np.sin(q2 + q3 + q4)),
         -np.cos(q1) * (l3 * np.sin(q2 + q3) + l4 * np.sin(q2 + q3 + q4)),
         -l4 * np.sin(q2 + q3 + q4) * np.cos(q1)],
        [np.cos(q1) * (l3 * np.cos(q2 + q3) + l2 * np.cos(q2) + l4 * np.cos(q2 + q3 + q4)),
         -np.sin(q1) * (l3 * np.sin(q2 + q3) + l2 * np.sin(q2) + l4 * np.sin(q2 + q3 + q4)),
         -np.sin(q1) * (l3 * np.sin(q2 + q3) + l4 * np.sin(q2 + q3 + q4)),
         -l4 * np.sin(q2 + q3 + q4) * np.sin(q1)],
        [0, l3 * np.cos(q2 + q3) + l2 * np.cos(q2) + l4 * np.cos(q2 + q3 + q4),
         l3 * np.cos(q2 + q3) + l4 * np.cos(q2 + q3 + q4),
         l4 * np.cos(q2 + q3 + q4)]
    ])
    return J

def Controler_pos(L, q, he, hdp, val):
    q1, q2, q3, q4 = q
    J = jacobiana_Brazo4DOF(L, q)
    K = val * np.eye(3)
    q1d = 0 * np.pi / 180
    q2d = 30 * np.pi / 180
    q3d = -15 * np.pi / 180
    q4d = +30 * np.pi / 180
    hq1 = q1d - q1
    hq2 = q2d - q2
    hq3 = q3d - q3
    hq4 = q4d - q4
    n = np.array([hq1, hq2, hq3, hq4])
    D = np.diag([1, 1, 5, 10])
    I = np.eye(4)
    TAREA_S = np.dot((I - np.dot(np.linalg.pinv(J), J)), np.dot(D, n))
    Vref = np.linalg.pinv(J) @ (K @ np.tanh(0.5 * he)) + TAREA_S
    Vref = np.clip(Vref, -5.0, 5.0) 
    return Vref

class ControladorNode(Node):
    def __init__(self):
        super().__init__('controlador')
        self.publisher_m1 = self.create_publisher(Float32, '/cmd_m1', 10)
        self.publisher_m2 = self.create_publisher(Float32, '/cmd_m2', 10)
        self.publisher_m3 = self.create_publisher(Float32, '/cmd_m3', 10)
        self.publisher_m4 = self.create_publisher(Float32, '/cmd_m4', 10)
        self.subscription = self.create_subscription(Joy, '/states', self.states_callback, 10)
        self.timer = self.create_timer(1.0 / 30, self.control_loop)
        self.q = np.zeros(4)
        self.q_p = np.zeros(4)
        self.h = np.zeros(3)
        self.u = np.zeros(4)
        self.a = True
        self.umbral = 0.01
        self.time_index = 0
        self.t_final = 60
        self.frec = 30
        self.t_s = 1 / self.frec
        self.t = np.arange(0, self.t_final, self.t_s)
        self.ref_1 = np.array([0.15, 0.15, 0.04])
        self.ref_2 = np.array([0.18, 0.20, 0.04])
        self.value = 9 / 3

    def states_callback(self, state_msg):
        self.q = [state_msg.axes[0], state_msg.axes[1], state_msg.axes[2], state_msg.axes[3]]
        self.q_p = [state_msg.axes[4], state_msg.axes[5], state_msg.axes[6], state_msg.axes[7]]

    def get_pose_arm(self):
        return self.q

    def get_vel_arm(self):
        return self.q_p

    def send_velocity_control(self, u):
        self.publisher_m1.publish(Float32(data=u[0]))
        self.publisher_m2.publish(Float32(data=u[1]))
        self.publisher_m3.publish(Float32(data=u[2]))
        self.publisher_m4.publish(Float32(data=u[3]))

    def control_loop(self):
        if self.time_index >= len(self.t):
            self.send_velocity_control([0, 0, 0, 0])
            return
        
        ref = self.ref_1 if self.a else self.ref_2
        self.q = self.get_pose_arm()
        self.q_p = self.get_vel_arm()
        self.h = CDArm4DOF(l, self.q)
        error = ref - self.h
        
        if np.linalg.norm(error) < self.umbral:
            self.a = False
        
        self.u = Controler_pos(l, self.q, error, np.zeros(3), 0.05)
        self.send_velocity_control(self.u)
        self.time_index += 1

def main(args=None):
    rclpy.init(args=args)
    controlador_node = ControladorNode()
    rclpy.spin(controlador_node)
    controlador_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
