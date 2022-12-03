#!/bin/sh
# `postCreate.sh` is called when the Dev Container is first created.
# It can be used for setup steps outside the Dockerfile.

. /opt/ros/$ROS_DISTRO/setup.sh

# Mitigates the Dockerfile somehow breaking folder permissions.
sudo chown user:user /code

# Something deleted the package indexes so we re-download them for convenience.
sudo apt-get update
sudo rosdep update
