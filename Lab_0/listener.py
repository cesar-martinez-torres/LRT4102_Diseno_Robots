#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__('listener_py')
        self.sub = self.create_subscription(
            String,
            'chatter',
            self.on_msg,
            10  # depth (QoS)
        )
        self.get_logger().info("Listener Python listo. Suscrito a /chatter")

    def on_msg(self, msg: String):
        self.get_logger().info(f'Recibido: "{msg.data}"')


def main():
    rclpy.init()
    node = Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()