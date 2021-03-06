#!/bin/sh

# postCreate.sh is called after the container is created by devcontainer.json
# It can be used to install & setup tools not wanted in the Dockerfile

# Install rosdep & build current packages for development mode
. /opt/ros/$ROS_DISTRO/setup.sh
# Something removed the apt cache so we have to get it back (rosdep uses apt to install some packages)
sudo apt-get update
sudo rosdep install -i --from-path /code -y
colcon build --symlink-install
