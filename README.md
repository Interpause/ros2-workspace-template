# ros2-workspace-template

Template for ROS2 workspace using [VS Code Dev Containers](https://code.visualstudio.com/docs/remote/containers) & [Docker Compose](https://docs.docker.com/compose/).

- Use `Dev Containers: Clone Repository in Container Volume...` in VS Code's Command Palette to auto-magically setup everything!
  - To re-open repository, either use `File > Open Recent` or repeat the above with the **exact SAME url**.
- For customization, do a global find & replace for `(OPTION)`.
  - Remember to `Dev Containers: Rebuild Container` afterwards!
- GUI apps are viewable via [noVNC](https://novnc.com/info.html) (VNC client web app) hosted on <http://localhost:6080/>.
  - **The password is `password`!**

## Table of Contents

- [Options](#options)
  - [Mount Point `/data`](#mount-point-data)
  - [VNC & RQT in Production Image](#vnc--rqt-in-production-image)
- [Other Features](#other-features)
  - [`requirements.txt` Escape Hatch](#requirementstxt-escape-hatch)
  - [VS Code Tasks](#vs-code-tasks)
  - [Docker Image Distribution](#docker-image-distribution)
  - [VS Code Extension Suggestions](#vs-code-extension-suggestions)
  - [Code Formatting](#code-formatting)
  - [Dev Container Lifecycle Hooks](#dev-container-lifecycle-hooks)
  - [Dev Container using Docker Compose](#dev-container-using-docker-compose)
- [Tips](#tips)
  - [`example_module`](#example_module)
  - [`git` Submodules](#git-submodules)
  - [Update Package Indexes](#update-package-indexes)
  - [Minimize Changes to the Dockerfile](#minimize-changes-to-the-dockerfile)
  - [Change ROS Distro](#change-ros-distro)
- [Troubleshooting](#troubleshooting)

## Options

`(OPTION)` marks key options to consider before creating the Dev Container. Some of the `(OPTION)`s work together to activate the features listed below:

### Mount Point `/data`

> Note: If using `Dev Containers: Clone Repository in Container Volume...`, the host folder path must be absolute. It is possible to use `$USERPROFILE` (Windows) or `$HOME` (Linux) for absolute paths. Otherwise, the host folder path would be relative to the Docker Compose file location.

Mount points are used to mount a folder from the host into the container. An example for mounting a folder to `/data` is included in [`docker-compose.dev.yml`](./docker-compose.dev.yml) as an `(OPTION)`. `/data` can be used to share anything from the host with the container (e.g., config files, models, databases). Also remember to:

- Add mount points to [`.gitignore`](./.gitignore) and [`.dockerignore`](./.dockerignore).
- Symlink mount points into your workspace folder for convenience in [`postCreate.sh`](./.devcontainer/hooks/postCreate.sh).

While one mount point is sufficient, more can be added by following `/data`'s example. See <https://docs.docker.com/storage/bind-mounts/> for more info.

### VNC & RQT in Production Image

By default, noVNC & RQT are not installed in the production image to save space. It is recommended to use ROS CLI or create a proper API instead to manage ROS systems. However, they can be enabled by following `(OPTION)`s in [`Dockerfile`](./Dockerfile). This can be convenient for early stages of deployment.

## Other Features

### `requirements.txt` Escape Hatch

Some dependencies may be unavailable from the `rosdep` package manager (check [ROS Index](https://index.ros.org)). For Python dependencies, they should be added to a `requirements.txt` created within the ROS package. The ROS package's `requirements.txt` should then be composed into the workspace's [`requirements.txt`](./requirements.txt) (example in [`requirements.txt`](./requirements.txt)). For other dependencies, they should be added to both Dockerfiles. See <https://github.com/ros/rosdistro/blob/master/CONTRIBUTING.md#rosdep-rules-contributions> with regards to adding new packages to `rosdep`.

### VS Code Tasks

For developer convenience, some common tasks are present in [`tasks.json`](./.vscode/tasks.json). Use them by opening the Command Palette and typing `task`. The following tasks are available:

- `rosdep install dependencies`: Install all dependencies of the current workspace.
- `colcon build all`: Build all (detected) ROS packages.
- `colcon build specific`: Build a specific ROS package.
- `update package index`: Update Ubuntu and ROS package indexes.

### Docker Image Distribution

See <https://docs.docker.com/engine/reference/commandline/docker/> for more info.

#### Building

> Note: If using `Dev Containers: Clone Repository in Container Volume...` on Windows, follow <https://code.visualstudio.com/docs/containers/choosing-dev-environment#_windows-subsystem-for-linux> to ensure built images are stored on the host computer's image repository.

```sh
docker build . -t organization/repo:vx.x.x -t organization/repo:latest
```

Images can have multiple names tagged to them. Tagging images with the version number and as latest helps when distributing images.

#### Exporting

> Note: Run this command on the host computer rather than in the Dev Container.

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

### Dev Container using Docker Compose

Instead of using Dev Container with a Dockerfile, this template uses it with Docker Compose instead. This is done for a few reasons:

- Include example of how to configure Docker for ROS to communicate across containers.
- Docker Compose is closer to real world deployment scenarios.
  - It is also more flexible and easier to configure than [`devcontainer.json`](./.devcontainer/devcontainer.json).

## Tips

### `example_module`

`example_module`'s [README.md](https://github.com/Interpause/ros-example-node/blob/main/README.md) provides instructions on how to create a `git` submodule and add ROS packages to it. It also contains a simple pub-sub `launch` file that can be used to test bandwidth & latency. To remove `example_module`:

```sh
git rm example_module
```

### `git` Submodules

> Note: The template will clone missing submodules for you by default.

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
apt-get update
rosdep update
```

### Minimize changes to the Dockerfile

When a step in the Dockerfile is changed, subsequent steps are invalidated and need to be rebuilt. Furthermore, ROS workspaces with the same Dockerfile share cache, resulting in less disk usage and shorter rebuilds.

The current Dockerfiles should work in most cases without needing edits. Nonetheless, if the Dockerfile becomes too complex, re-organizing it is fine. See <https://docs.docker.com/develop/develop-images/dockerfile_best-practices/> for more info on Docker layer caching and other Docker tips.

### Change ROS Distro

To change ROS Distro, do a global search for the current distro (`humble`) and replace as necessary with the new distro. Afterwards, rebuild the Dev Container.

## Troubleshooting

### Setup Issues

- On first setup, the Dev Container fails to build and complains about `/r`.
  - The Dev Container lifecycle hooks were checked out with CRLF instead of LF line endings.
  - Do `git config --global core.autocrlf input` and re-clone the repository.
  - See <https://code.visualstudio.com/docs/remote/troubleshooting#_resolving-git-line-ending-issues-in-wsl-resulting-in-many-modified-files> for more info.
- When modifying the Dockerfile, earlier cached steps might be stale, causing later steps to fail.
  - Rebuild the Dev Container without cache.

### Build Issues

- Try deleting the `build` and `install` folders before rebuilding all packages.

### Runtime Issues

- `rosdep` has no version lock, which means there is no protection against breaking changes when packages update.
  - See: <https://github.com/ros-infrastructure/rosdep/issues/325>
  - One solution would be to use your own install script instead of `rosdep`.
- ROS `launch` files aren't symlinked unlike Python code.
  - Rebuild the package when `launch` files are modified.
