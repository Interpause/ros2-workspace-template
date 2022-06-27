from __future__ import annotations
from collections import deque
from dataclasses import dataclass, field
import sys
import time

import rclpy
from rclpy.qos import QoSPresetProfiles
from rclpy.node import Node
from std_msgs.msg import String

from pyratetest.job import Job, JobCfg

# this is the name that displays in debugging tools
NODE_NAME = "sub"


@dataclass
class SubscriberCfg(JobCfg):
    topic: str = "topic"
    """Topic to publish to."""
    buf_size: int = 1000
    """Maximum size of rate timings buffer."""
    buf_duration: int = 2
    """Maximum duration in seconds of rate timings buffer."""
    max_rate: int = 0.5


@dataclass
class Subscriber(Job[SubscriberCfg]):
    """Simple subscriber."""

    ini_cfg: SubscriberCfg = field(default_factory=SubscriberCfg)

    def attach_params(self, node, cfg: SubscriberCfg):
        super(Subscriber, self).attach_params(node, cfg)

        node.declare_parameter("topic", cfg.topic)
        node.declare_parameter("buf_size", cfg.buf_size)
        node.declare_parameter("buf_duration", cfg.buf_duration)

        self._buf = deque([], cfg.buf_size)

    def attach_behaviour(self, node, cfg: SubscriberCfg):
        super(Subscriber, self).attach_behaviour(node, cfg)

        self._subscriber = node.create_subscription(
            String, cfg.topic, self._listen_cb, QoSPresetProfiles.SENSOR_DATA.value
        )

        self._timer = node.create_timer(1.0 / cfg.max_rate, self._fps_timer_cb)

        self.log.info(
            f'Listening on "{cfg.topic}" and reporting rate at {cfg.max_rate}Hz.'
        )

    def detach_behaviour(self, node):
        super(Subscriber, self).detach_behaviour(node)

        node.destroy_subscription(self._subscriber)
        node.destroy_timer(self._timer)

    def _fps_timer_cb(self):
        self.log.info(f"{len(self._buf)/sum(self._buf) if self._buf else 0:.1f}Hz")

    def _listen_cb(self, msg: String):
        now = time.time()
        try:
            self._buf.append(now - self._prev_time)
        except AttributeError:
            pass
        self._prev_time = now

        while sum(self._buf) >= self.cfg.buf_duration and len(self._buf) > 1:
            self._buf.popleft()

    def on_params_change(self, node, changes):
        self.log.info(f"Config changed: {changes}.")
        if any(n in changes for n in ("buf_size", "topic", "max_rate")):
            self.log.info(f"Config change requires restart. Restarting...")
            self.restart()
        return True


def main(args=None):
    if __name__ == "__main__" and args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = Node(NODE_NAME)

    cfg = SubscriberCfg(topic="hello_topic")
    Subscriber(node, cfg)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
