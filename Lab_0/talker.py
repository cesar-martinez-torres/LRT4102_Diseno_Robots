#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('talker_py')  # nombre del nodo
        self.pub = self.create_publisher(String, 'chatter', 10)  # tópico + "queue"
        self.counter = 0
        self.timer = self.create_timer(0.5, self.on_timer)  # 2 Hz
        self.get_logger().info("Talker Python listo. Publicando en /chatter")

    def on_timer(self):
        msg = String()
        msg.data = f"Hello from lab_0 (Python): {self.counter}"
        self.pub.publish(msg)
        self.get_logger().info(f'Publicado: "{msg.data}"')
        self.counter += 1


def main():
    rclpy.init()
    node = Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()