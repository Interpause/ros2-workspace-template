from __future__ import annotations
from dataclasses import dataclass

from rclpy.node import Node


@dataclass(eq=False)
class NiceNode(Node):
    """Extension of Node with some common properties."""

    node_name: str = "undefined"
    """name of node"""
    max_fps: int = 30
    """max rate of node"""

    def __post_init__(self):
        super(NiceNode, self).__init__(self.node_name)

        self._log = self.get_logger()
