#!/bin/sh
# `postStart.sh` is called whenever the Dev Container starts.
# It can be used for misc tasks (e.g., ensuring dependencies are installed).

. /opt/ros/$ROS_DISTRO/setup.sh

# Due to https://github.com/microsoft/vscode-remote-release/issues/6683,
# we have to disable git's repository trust feature.
# Due to https://github.com/microsoft/vscode-remote-release/issues/6810#issuecomment-1159354677,
# this cannot be done in the Dockerfile (else VS Code doesn't configure `.gitconfig`).
git config --global safe.directory "*"

# Ensure submodules are cloned; Doesn't affect already cloned ones.
git submodule update --init --recursive

# (OPTION) Symlink `/data` mount point to workspace folder for convenience.
# ln -sf /data "$WORKSPACE_ROOT/"

# Ensure dependencies are installed.
rosdep install --ignore-src --from-path "$WORKSPACE_ROOT" -y
pip install -r "$WORKSPACE_ROOT/requirements.txt"
