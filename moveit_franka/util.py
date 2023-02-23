from rclpy.node import Node


class Subscriber(Node):

    def __init__(self, name, msg, topic):
        super().__init__('%s_subscriber' % name)
        self.subscriber = self.create_subscription(
            msg,
            topic,
            self.update_callback,
            qos_profile=1,
        )
        self.current = None

    def update_callback(self, msg):
        self.current = msg

    def get_current_msg(self):
        return self.current