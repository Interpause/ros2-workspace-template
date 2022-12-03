# ros2-workspace-template

Template for ROS2 workspace using [VS Code Dev Containers](https://code.visualstudio.com/docs/remote/containers) & [Docker Compose](https://docs.docker.com/compose/).

- **Before creating the Dev Container**, do a global search for `(OPTION)` and read what they say!
- GUI apps are viewable via [noVNC](https://novnc.com/info.html) (VNC client web app) hosted on <http://localhost:6080/>.
  - **The password is `password`!**

## Table of Contents

- [Options](#options)
  - [Store Repository in Named Volume](#store-repository-in-named-volume)
  - [Mount Point `/data`](#mount-point-data)
  - [VNC & RQT in Production Image](#vnc--rqt-in-production-image)
  - [Native GUI Apps via X11](#native-gui-apps-via-x11)
- [Other Features](#other-features)
  - [`requirements.txt` Escape Hatch](#requirementstxt-escape-hatch)
  - [VS Code Tasks](#vs-code-tasks)
  - [Docker Image Distribution](#docker-image-distribution)
  - [VS Code Extension Suggestions](#vs-code-extension-suggestions)
  - [Code Formatting](#code-formatting)
  - [Dev Container Lifecycle Hooks](#dev-container-lifecycle-hooks)
- [Tips](#tips)
  - [`git` Submodules](#git-submodules)
  - [Update Package Indexes](#update-package-indexes)
  - [Minimize Changes to the Dockerfile](#minimize-changes-to-the-dockerfile)
  - [Change ROS Distro](#change-ros-distro)
- [Troubleshooting](#troubleshooting)

## Options

`(OPTION)` marks key options to consider before creating the Dev Container. `(LINUX)` and `(X11)` indicate options that are valid only if your host OS is Linux, or you use X11 on Linux. Some of the `(OPTION)`s work together to activate the features listed below:

### Store Repository in Named Volume

On Windows or MacOS, using a named volume to store the repository fixes slow build times and avoids certain issues. The steps for this feature are:

1. ([`docker-compose.dev.yml`](./docker-compose.dev.yml)) Create a named volume to store the repository.
2. ([`docker-compose.dev.yml`](./docker-compose.dev.yml)) Mount the named volume to `/code` where the repository should be.
3. ([`postStart.sh`](./.devcontainer/hooks/postStart.sh)) Clone the repository into the named volume.

See <https://code.visualstudio.com/remote/advancedcontainers/improve-performance#_use-a-named-volume-for-your-entire-source-tree> for more info.

### Mount Point `/data`

Mount points are used to mount a folder from the host into the container. By following `(OPTION)`s, `./data` on the host is mounted to `/data` in the container. `/data` can be used to share anything from the host with the container (e.g., config files, models, databases).

While one mount point is sufficient, more can be added by following `/data`'s example. See <https://docs.docker.com/storage/bind-mounts/> for more info.

### VNC & RQT in Production Image

By default, noVNC & RQT are not installed in the production image to save space. It is recommended to use ROS CLI or create a proper API instead to manage ROS systems. However, they can be enabled by following `(OPTION)`s in [`Dockerfile`](./Dockerfile). This can be convenient for early stages of deployment.

### Native GUI Apps via X11

For Linux users, it is possible for GUI apps in the container to show natively on the host if using X11 instead of Wayland. For Windows users, it is also possible via software like [VcXsrv](https://sourceforge.net/projects/vcxsrv/).

## Other Features

### `requirements.txt` Escape Hatch

Some dependencies may be unavailable from the `rosdep` package manager (check [ROS Index](https://index.ros.org)). For Python dependencies, they should be added to a `requirements.txt` created within the ROS package. The ROS package's `requirements.txt` should then be composed into the workspace's [`requirements.txt`](./requirements.txt) (example in [`requirements.txt`](./requirements.txt)). For other dependencies, they should be added to both Dockerfiles. See <https://github.com/ros/rosdistro/blob/master/CONTRIBUTING.md#rosdep-rules-contributions> with regards to adding new packages to `rosdep`.

### VS Code Tasks

For developer convenience, some common tasks such as building packages and installing dependencies are present in [`tasks.json`](./.vscode/tasks.json). Use them by opening the command bar and typing `task`.

### Docker Image Distribution

See <https://docs.docker.com/engine/reference/commandline/docker/> for more info.

#### Building

```sh
docker build . -t organization/repo:vx.x.x -t organization/repo:latest
```

Images can have multiple names tagged to them. Tagging images with the version number and as latest helps when distributing images.

#### Exporting

```sh
docker save organization/repo:vx.x.x organization/repo:latest -o repo-vx.x.x.tar
```

Compressing the image afterwards using `xzip` is recommended to save space. Docker is able to load compressed images (i.e., `repo-vx.x.x.tar.xz`) without decompressing manually.

#### Importing

```sh
docker load -i repo-vx.x.x.tar.xz
```

Imports the image and its names. It will still be tagged as `organization/repo:vx.x.x` and `organization/repo:latest`, conveniently replacing the previous `latest`.

### VS Code Extension Suggestions

Both [`devcontainer.json`](./.devcontainer/devcontainer.json) and [`extensions.json`](./.vscode/extensions.json) are configured such that VS Code will automatically install some extensions. These are highly recommended for developer experience.

### Code Formatting

[`black`](https://github.com/psf/black) is used to format Python code due to its uncompromising design. Imports are also automatically sorted using compatible rules. See [`settings.json`](./.vscode/settings.json) for the configuration.

### Dev Container Lifecycle Hooks

[`devcontainer.json`](./.devcontainer/devcontainer.json) allows adding lifecycle hooks which can be useful for special cases. See the [`hooks`](./.devcontainer/hooks/) folder for some examples.

## Tips

### `git` Submodules

It is recommended to add custom ROS packages to the workspace via [`git` Submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules). However, when cloning a repository using `git`, it will not clone submodules by default. In order to clone submodules, use:

```sh
git clone --recurse-submodules
```

If the repository is already cloned and has empty submodules, do:

```sh
git submodule update --init --recursive
```

If you want `git` to clone submodules by default, try:

```sh
git config --global submodule.recurse true
```

Otherwise, use [Github Desktop](https://desktop.github.com/), which uses `--recurse-submodules` by default.

Nonetheless, when submodules are cloned, they will be in a detached state, which prevents committing. Switching submodules to the main branch can either be done through VS Code's Source Control tab, or:

```sh
git submodule foreach --recursive git checkout main
```

This is done for you automatically if using a named volume to store the repository.

### Update Package Indexes

The `rosdep` and Ubuntu package managers rely on a local cache of their package index. If the package index is outdated, it may not contain any active package distribution server, leading to package downloads failing. Hence, it is recommended to periodically re-download the package index:

```sh
sudo apt-get update
sudo rosdep update
```

### Minimize changes to the Dockerfile

When a step in the Dockerfile is changed, subsequent steps are invalidated and need to be rebuilt. Furthermore, ROS workspaces with the same Dockerfile share cache, resulting in less disk usage and shorter rebuilds.

The current Dockerfiles should work in most cases without needing edits. Nonetheless, if the Dockerfile becomes too complex, re-organizing it is fine.

### Change ROS Distro

To change ROS Distro, do a global search for the current distro (`humble`) and replace as necessary with the new distro. Afterwards, rebuild the Dev Container.

## Troubleshooting

- `rosdep` has no version lock, see: <https://github.com/ros-infrastructure/rosdep/issues/325>.
  - One solution would be to use your own install script instead of `rosdep`.
- Delete both the `build` and `install` folder and rebuild everything.
- While Python code is symlinked, the ROS `launch` files aren't, meaning rebuilding the _specific_ package is needed when `launch` files are changed.
- Rebuild the container without cache.
