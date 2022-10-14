#!/bin/sh

# postStart.sh is called every time the container starts by devcontainer.json
# It can be for misc tasks like ensuring all dependencies are installed

# Due to https://github.com/microsoft/vscode-remote-release/issues/6683
# we have to add workspace folder as trusted by git
# Due to https://github.com/microsoft/vscode-remote-release/issues/6810#issuecomment-1159354677
# this cannot be done in Dockerfile (else VSCode fails to configure git in the container)
git config --global safe.directory "*"

# Ensure all dependencies are installed
. /opt/ros/$ROS_DISTRO/setup.sh

# Needed if using named volume to store repo
test -d "/code/.git" \
  || git clone "insert repo url" /code --recurse-submodules -j8

sudo rosdep install -i --from-path /code -y
sudo pip install -r /code/requirements.txt
