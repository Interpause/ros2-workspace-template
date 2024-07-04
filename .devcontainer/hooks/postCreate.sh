#!/bin/sh
# `postCreate.sh` is called when the Dev Container is first created.
# It can be used for setup steps outside the Dockerfile.

# Allow apt index & cache to be kept.
sudo rm -f /etc/apt/apt.conf.d/docker-clean; \
  echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' | sudo tee /etc/apt/apt.conf.d/keep-cache

# Fix perms issues.
sudo chown -R vscode:vscode ~/.cache

# Auto-activate ROS whenever shell is opened.
echo "source /opt/ros/$ROS_DISTRO/setup.zsh" >> ~/.zshrc

# Ensure submodules are cloned; Doesn't affect already cloned ones.
git submodule update --init --recursive

# Ensure dependencies are installed.
sudo apt-get update
rosdep update
rosdep install --ignore-src --from-path . -y
pip install -r requirements.txt

# Fix RQT icons.
mkdir ~/.icons && ln -s /usr/share/icons/Tango ~/.icons/hicolor

# postCreate.sh
sudo rm -f /etc/apt/apt.conf.d/docker-clean; \
  echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' | sudo tee /etc/apt/apt.conf.d/keep-cache
