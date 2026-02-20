#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtleController(Node):
    """
    Nodo controlador básico para turtlesim.

    Publica comandos de velocidad en /turtle1/cmd_vel
    usando el mensaje geometry_msgs/msg/Twist.

    Por ahora implementa un patrón simple:
    avanzar + girar → movimiento circular.
    """

    def __init__(self):
        super().__init__('turtle_controller')

        # Publisher hacia el tópico de control de turtlesim
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Timer que ejecuta la función de control a 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        # Velocidades básicas
        self.linear_speed = 2.0
        self.angular_speed = 1.0

        self.get_logger().info("TurtleController iniciado. Controlando /turtle1/cmd_vel")

    def control_loop(self):
        msg = Twist()

        # Movimiento simple: avanzar y girar
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed

        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = TurtleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()