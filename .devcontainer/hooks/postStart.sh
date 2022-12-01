#!/bin/sh

# postStart.sh is called every time the container starts by devcontainer.json.
# It can be for misc tasks like ensuring all dependencies are installed.

# Due to https://github.com/microsoft/vscode-remote-release/issues/6683
# we have to explicitly trust workspace folder for git.
# Due to https://github.com/microsoft/vscode-remote-release/issues/6810#issuecomment-1159354677
# this cannot be done in Dockerfile (else VSCode fails to configure git in the container).
git config --global safe.directory "*"

. /opt/ros/$ROS_DISTRO/setup.sh

# (Optional) Clone repo on first setup if using named volume to store repo.
test -d "/code/.git" \
  || ( \
    git clone "(Optional) Insert repo url" /code --recurse-submodules \
    && cd /code \
    && git submodule foreach --recursive git checkout main \
  )

# Ensure all dependencies are installed.
sudo rosdep install -i --from-path /code -y
sudo pip install -r /code/requirements.txt
