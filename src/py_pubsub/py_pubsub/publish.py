from __future__ import annotations
from dataclasses import dataclass, field
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from py_pubsub.job import Job, JobCfg

# TODO: see http://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html
# Use above to put Python utilities, C++ Utilities & Interfaces all in one package


# this is the name that displays in debugging tools
NODE_NAME = "minimal_publisher_test"


@dataclass
class PublisherCfg(JobCfg):
    topic: str = "topic"
    """Topic to publish to."""
    pub_msg: str = "Hello World"
    """Msg to publish."""


@dataclass
class Publisher(Job[PublisherCfg]):
    """Simple publisher."""

    ini_cfg: PublisherCfg = field(default_factory=PublisherCfg)

    def attach_params(self, node, cfg: PublisherCfg):
        super(Publisher, self).attach_params(node, cfg)

        self.topic = node.declare_parameter("topic", cfg.topic)
        self.pub_msg = node.declare_parameter("pub_msg", cfg.pub_msg)

    def attach_behaviour(self, node, cfg: PublisherCfg):
        super(Publisher, self).attach_behaviour(node, cfg)

        self._publisher = node.create_publisher(String, cfg.topic, 10)
        self._timer = node.create_timer(1.0 / cfg.max_rate, self._timer_cb)
        self._i = 0
        self.log.info(f'Publishing to "{cfg.topic}" at {cfg.max_rate}Hz.')

    def detach_behaviour(self, node):
        super(Publisher, self).detach_behaviour(node)

        node.destroy_publisher(self._publisher)
        node.destroy_timer(self._timer)

    def _timer_cb(self):
        msg = String()
        msg.data = f"[{self._i}] {self.cfg.pub_msg}"
        self._publisher.publish(msg)
        self.log.info(f"Publish: {msg.data}")
        self._i += 1

    def on_params_change(self, node, changes):
        self.log.info(f"Config changed: {changes}.")
        if "max_rate" in changes or "topic" in changes:
            self.log.info(f"Config change requires restart. Restarting...")
            self.restart()
        return True


def main(args=None):
    if __name__ == "__main__" and args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = Node(NODE_NAME)

    cfg = PublisherCfg(max_rate=5, topic="hello_topic", pub_msg="hey")
    Publisher(node, cfg)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
