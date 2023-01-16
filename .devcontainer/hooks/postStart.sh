#!/bin/sh
# `postStart.sh` is called whenever the Dev Container starts.
# It can be used for misc tasks (e.g., ensuring dependencies are installed).

. /opt/ros/$ROS_DISTRO/setup.sh

# Due to https://github.com/microsoft/vscode-remote-release/issues/6683,
# we have to disable git's repository trust feature.
# Due to https://github.com/microsoft/vscode-remote-release/issues/6810#issuecomment-1159354677,
# this cannot be done in the Dockerfile (else VS Code doesn't configure `.gitconfig`).
git config --global safe.directory "*"

# If using named volume to store repository, clone and setup the repository.
test -d "/code/.git" \
  || ( \
    git clone "(OPTION) Insert repository url" /code --recurse-submodules \
    && cd /code \
    && git submodule foreach --recursive git checkout main \
  )

# (OPTION) Symlink `/data` to `/code/data` for convenience if using `/data` mount point.
# ln -sf /data /code/

# Ensure dependencies are installed.
rosdep install -i --from-path /code -y
pip install -r /code/requirements.txt
