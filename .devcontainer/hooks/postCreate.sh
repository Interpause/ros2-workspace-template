#!/bin/sh

# postCreate.sh is called after the container is created by devcontainer.json.
# It can be used to install & setup tools not wanted in the Dockerfile.

# This mitigates the Dockerfile somehow breaking the folder permissions.
sudo chown user:user /code

. /opt/ros/$ROS_DISTRO/setup.sh
# Something removed the package indexes so we download them again for convenience.
sudo apt-get update
sudo rosdep update
