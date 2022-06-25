# ros2-workspace-template

Template for ROS2 ([Humble Hawksbill](https://docs.ros.org/en/humble/)) workspace using [VSCode Remote Container](https://code.visualstudio.com/docs/remote/containers) with [Docker Compose](https://docs.docker.com/compose/). Go to <http://localhost:6080/> in order to view the development container's desktop (for GUI apps).

## Conventions

### `Dockerfile` Naming

`Dockerfile` may be suffixed when multiple images (with different dependencies) are built within a project. For example, `Dockerfile.ui` for the production UI image, `Dockerfile.talker` for the ROS2 talker node, and `Dockerfile.listener` for the ROS2 listener node. If there is only one image or a "main image", `Dockerfile` should be unsuffixed.

For development, ideally a single `Dockerfile.dev` that is comprehensive (i.e. having all dependencies) should be used (to minimize rebuilding & wasted space for multiple images). To test multiple ROS2 nodes, it is better to just change the command used in `docker-compose.dev.yml` (for example, `/bin/sh -c "source install/local_setup.sh && ros2 run pubsub talker").

### `docker-compose.yml` Naming

`docker-compose.yml` may be suffixed for special configurations. For example, `docker-compose.dev.yml` for development, `docker-compose.ci.yml` for testing, or `docker-compose.prod.yml` for production. `docker-compose.yml` unsuffixed should be the base/common configuration.

## Using GUI Apps

For ease of usage, the VNC approach is default over the X Server approach.

### Connecting to VNC

Using VSCode's [Remote Containers](https://code.visualstudio.com/docs/remote/containers) is necessary due to reliance on [Dev Container Features](https://code.visualstudio.com/docs/remote/containers#_dev-container-features-preview), specifically their [Desktop Install Script](https://github.com/microsoft/vscode-dev-containers/blob/main/script-library/docs/desktop-lite.md). The configuration can be found in [`.devcontainer/devcontainer.json`](.devcontainer/devcontainer.json):

```jsonc
{
  // ...
  "features": {
    "desktop-lite": {
      "password": "password",
      "webPort": "6080",
      "vncPort": "5901"
    }
    // ...
  }
}
```

Afterwards, go to <http://localhost:6080/> to view the container's desktop, or use a VNC client of your choice.

### Using Host's X Server

#### Linux

Within [`docker-compose.dev.yml`](docker-compose.dev.yml), there are comments such as the below:

```yml
# (Linux only) (X Server) Use X Server address from host
# environment:
#   DISPLAY: ${DISPLAY}
```

Uncomment these in order for GUI apps in the container to use the host Linux's X Server.

#### Windows

On Windows, installing an X Server such as [VcXsrv](https://sourceforge.net/projects/vcxsrv/) is required. You will also need to set the `DISPLAY` environment. For example, insert the below into [`.devcontainer/hooks/postStart.sh`](.devcontainer/hooks/postStart.sh):

```sh
# postStart.sh
echo "export DISPLAY=host.docker.internal:0.0" >> ~/.bashrc
```

If using a high-resolution display, ROS Qt might appear small, in which case adjusting `QT_SCALE_FACTOR` might help:

```sh
# postStart.sh
echo "export QT_SCALE_FACTOR=2" >> ~/.bashrc
```