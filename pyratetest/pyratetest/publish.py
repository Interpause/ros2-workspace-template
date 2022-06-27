from __future__ import annotations
from dataclasses import dataclass, field
import sys

import rclpy
from rclpy.qos import QoSPresetProfiles
from rclpy.node import Node
from std_msgs.msg import String

from pyratetest.job import Job, JobCfg

# this is the name that displays in debugging tools
NODE_NAME = "pub"


@dataclass
class PublisherCfg(JobCfg):
    topic: str = "topic"
    """Topic to publish to."""
    pub_size: int = 10
    """Size of msg to publish."""


@dataclass
class Publisher(Job[PublisherCfg]):
    """Simple publisher."""

    ini_cfg: PublisherCfg = field(default_factory=PublisherCfg)

    def attach_params(self, node, cfg: PublisherCfg):
        super(Publisher, self).attach_params(node, cfg)

        node.declare_parameter("topic", cfg.topic)
        node.declare_parameter("pub_size", cfg.pub_size)

    def attach_behaviour(self, node, cfg: PublisherCfg):
        super(Publisher, self).attach_behaviour(node, cfg)

        self._msg = "A" * cfg.pub_size
        self._publisher = node.create_publisher(
            String, cfg.topic, QoSPresetProfiles.SENSOR_DATA.value
        )
        try:
            self._timer = node.create_timer(1.0 / cfg.max_rate, self._timer_cb)
        except ZeroDivisionError:
            pass
        self.log.info(
            f'Publishing {cfg.pub_size} chars to "{cfg.topic}" at {cfg.max_rate}Hz.'
        )

    def detach_behaviour(self, node):
        super(Publisher, self).detach_behaviour(node)

        node.destroy_publisher(self._publisher)
        node.destroy_timer(self._timer)

    def _timer_cb(self):
        msg = String()
        msg.data = self._msg
        self._publisher.publish(msg)

    def on_params_change(self, node, changes):
        self.log.info(f"Config changed: {changes}.")
        if any(n in changes for n in ("max_rate", "topic", "pub_size")):
            self.log.info(f"Config change requires restart. Restarting...")
            self.restart()
        return True


def main(args=None):
    if __name__ == "__main__" and args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = Node(NODE_NAME)

    cfg = PublisherCfg(max_rate=5, topic="hello_topic", pub_size=100)
    Publisher(node, cfg)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
