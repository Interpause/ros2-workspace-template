from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Feels like crappier version of click
PACKAGE_NAME = "pyratetest"


# See documentation: https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        description="Set the namespace of the nodes",
        default_value=PACKAGE_NAME,
    )

    pub1_node = Node(
        package=PACKAGE_NAME, namespace=namespace, executable="pub", name="publisher1"
    )
    pub2_node = Node(
        package=PACKAGE_NAME, namespace=namespace, executable="pub", name="publisher2"
    )

    sub_node = Node(
        package=PACKAGE_NAME, namespace=namespace, executable="sub", name="subscriber"
    )

    return LaunchDescription([namespace_arg, pub1_node, pub2_node, sub_node])

