from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Generic, TypeVar
from dataclasses import dataclass, field, replace

from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import (
    ParameterDescriptor,
    FloatingPointRange,
    SetParametersResult,
)


@dataclass
class JobCfg:
    """Default config for Job."""

    max_rate: int = 30
    """Max rate in Hertz for job."""
    use_sim_time: bool = False
    """ROS Node param for using another clock; i.e. to run simulations."""


CT = TypeVar("CT", bound=JobCfg)
"""Generic for class that extends JobCfg."""


@dataclass
class Job(ABC, Generic[CT]):
    """Base class of common properties for Jobs."""

    # Although it is fields of Job, I still pass node, cfg, etc to the
    # function. It makes the functions clearer.
    # cfg vs params: params are from ROS Node, cfg is dataclass mirroring params

    node: Node = field(default_factory=Node)
    """Node to attach Job to."""
    ini_cfg: CT = field(default_factory=JobCfg)
    """Initial/default config for Job."""

    @property
    def cfg(self) -> CT:
        """Current (always updated) config of Job."""

        if self._cfg is None:
            self._cfg = type(self.ini_cfg)(
                **{n: p.value for n, p in self.node._parameters.items()}
            )
        return self._cfg

    @abstractmethod
    def attach_params(self, node: Node, cfg: CT):
        """Attaches Job config to Node as parameters."""

        node.declare_parameter("max_rate", float(cfg.max_rate))
        node.set_descriptor(
            "max_rate",
            ParameterDescriptor(
                description="Max rate in Hertz for job.",
                type=Parameter.Type.DOUBLE.value,
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=32767.0, step=0.0)
                ],
            ),
        )

    @abstractmethod
    def attach_behaviour(self, node: Node, cfg: CT):
        """Attaches Job behaviours like pub-sub, services, etc to Node."""
        pass

    @abstractmethod
    def detach_behaviour(self, node: Node):
        """Detach & clean up Job behaviours attached to Node."""
        pass

    @abstractmethod
    def on_params_change(self, node: Node, changes: dict):
        """Handle change in parameters.

        Args:
            node (Node): ROS Node
            params (dict): Map of parameter name to new value

        Returns:
            Union[Tuple[bool, str], bool]: Whether the change is successful & optionally a message.
        """
        return True, "NIL"

    def params_to_cfg(self, params: list[Parameter]):
        """Convert list of Parameter to dict (Partial of JobCfg)."""
        return {p.name: p.value for p in params}

    def restart(self):
        """Restart Job, used for example to make config changes effective."""
        self.detach_behaviour(self.node)
        self.attach_behaviour(self.node, self.cfg)

    def _param_change_cb(self, params: list[Parameter]):
        # ros2 only includes the params that were changed.
        changes = self.params_to_cfg(params)
        # apply changes NOW
        self._cfg = replace(self._cfg, **changes)

        success = self.on_params_change(self.node, changes)
        if isinstance(success, tuple):
            success, msg = success
        else:
            msg = "NIL"
        # revert config by retrieving from ROS params (which havent changedyet)
        if not success:
            self._cfg = None
        return SetParametersResult(successful=success, reason=msg)

    def __post_init__(self):
        """Attaches Job to Node."""

        self.log = self.node.get_logger()
        # current cfg based off node params for easier access
        # set to None whenever cfg needs to be recalculated
        # use self.cfg getter to get current cfg
        self._cfg = None

        # use self.ini_cfg to set params, which then sets self._cfg
        self.attach_params(self.node, self.ini_cfg)
        # TODO: check that params can be successfully loaded from ROS save file
        self.attach_behaviour(self.node, self.cfg)
        # NOTE: till all callbacks return True, ros won't actually change the
        # params. This can cause glitches such as stale config.
        self.node.add_on_set_parameters_callback(self._param_change_cb)
