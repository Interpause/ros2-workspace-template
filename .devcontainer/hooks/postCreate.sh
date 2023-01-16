#!/bin/sh
# `postCreate.sh` is called when the Dev Container is first created.
# It can be used for setup steps outside the Dockerfile.

. /opt/ros/$ROS_DISTRO/setup.sh

# Mitigates the Dockerfile somehow breaking folder permissions.
echo Workspace root is $WORKSPACE_ROOT
sudo ln -s "$WORKSPACE_ROOT" /code
sudo chown user:user "$WORKSPACE_ROOT"

# Something deleted the package indexes so we re-download them for convenience.
sudo apt-get update
sudo rosdep update
