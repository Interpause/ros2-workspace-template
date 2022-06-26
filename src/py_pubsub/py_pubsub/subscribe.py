from __future__ import annotations
from dataclasses import dataclass
import sys

import rclpy
from std_msgs.msg import String

from py_pubsub.old import NiceNode


@dataclass(eq=False)
class Subscriber(NiceNode):
    """Simple subscriber."""

    topic: str = "topic"
    """topic to listen to"""

    def __post_init__(self):
        super(Subscriber, self).__post_init__()

        self._subscriber = self.create_subscription(
            String, self.topic, self._listen_cb, 10
        )

    def _listen_cb(self, msg: String):
        self._log.info(f"Received: {msg.data}")


def main(args=None):
    if __name__ == "__main__" and args is None:
        args = sys.argv

    rclpy.init(args=args)

    subscriber = Subscriber(
        node_name="minimal_subscriber_test",
        # max_fps=5,
        topic="hello_topic",
    )

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
