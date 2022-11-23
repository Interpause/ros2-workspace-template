# ros2-workspace-template

Template for ROS2 workspace using [VSCode Remote Container](https://code.visualstudio.com/docs/remote/containers) with [Docker Compose](https://docs.docker.com/compose/). Go to <http://localhost:6080/> in order to view the development container's desktop (for GUI apps).

## Customizing

All parts of the template to pay attention to have been marked with `(Optional)`. Doing a global search for it should guide you in customizing the various builtin features.

### Additional Disk Performance on Windows

Following <https://code.visualstudio.com/remote/advancedcontainers/improve-performance#_use-a-named-volume-for-your-entire-source-tree>, it is possible to use a named volume to store the entire repository to improve disk performance on Windows. This resolves the longer ROS build times due to the inefficiencies of bind mounting. To use a named volume, there are instructions in [`postCreate.sh`](.devcontainer/hooks/postCreate.sh) and [`docker-compose.dev.yml`](docker-compose.dev.yml).

## Testing

Run `ros2 run pyratetest pub` and `ros2 run pyratetest sub` in two separate terminals. Use `rqt` (see GUI section [below](#using-gui-apps)) to reconfigure both nodes live (located under Plugins > Configuration > Dynamic Reconfigure). Alternatively, use the launch file via `ros2 launch pyratetest test.launch.py`. Use `--show-args` to see the launch options.

## Docker Image Distribution

### Building

```sh
docker build . -t example/example:vx.x.x -t example/example:latest
```

Each Docker image can have multiple names associated to it. In the above, tagging the image by both the version number and as latest helps for distributing images.

### Exporting

```sh
docker save example/example:vx.x.x example/example:latest -o example.tar
```

Using compression algorithms like `gzip` or `xzip` is recommended to save space. Docker is able to load compressed images (i.e. `example.tar.xz`) without manually decompressing first.

### Importing

```sh
docker load -i example.tar.xz
```

Imports the Docker image and its names. It will still be tagged as `example/example:vx.x.x` and `example/example:latest`, conveniently replacing the old image that was tagged as `latest`.

## Using GUI Apps

For ease of usage, the VNC approach is default over the X Server approach. ~~However, the X Server approach is more performant.~~ The X Server approach is only more performant on Linux, whereas on Windows, the VNC approach performs better. However, the X Server approach has a more native feel.

### Connecting to VNC

Go to <http://localhost:6080/> to view the container's desktop via the noVNC web client, or use a VNC client of your choice.

### Using Host's X Server

#### Linux

Within [`docker-compose.dev.yml`](docker-compose.dev.yml), there are comments such as the below:

```yml
# (Optional)(Linux only)(X Server) Use X Server address from host.
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

If using a high-resolution display, `rqt` might appear small, in which case adjusting `QT_SCALE_FACTOR` might help:

```sh
# postStart.sh
echo "export QT_SCALE_FACTOR=2" >> ~/.bashrc
```

## Additional Tips

### Update `rosdistro` Ocassionally

Run `sudo rosdep update` to update the package index.

## Troubleshooting

- `rosdep` has no version lock, see: <https://github.com/ros-infrastructure/rosdep/issues/325>.
  - One solution would be to use your own install script instead of `rosdep`.
- Delete both the `build` and `install` folder and rebuild everything.
- While Python code is symlinked, the `launch` files aren't, meaning rebuilding the _specific_ package is needed when `launch` files are changed.
- In the worst case scenario, rebuild the container without cache.

## Conventions

### `Dockerfile` Naming

`Dockerfile` may be suffixed when multiple images (with different dependencies) are built within a project. For example, `Dockerfile.ui` for the production UI image, `Dockerfile.talker` for the ROS2 talker node, and `Dockerfile.listener` for the ROS2 listener node. If there is only one image or a "main image", `Dockerfile` should be unsuffixed.

For development, ideally a single `Dockerfile.dev` that is comprehensive (i.e. having all dependencies) should be used (to minimize rebuilding & wasted space for multiple images).

### `docker-compose.yml` Naming

`docker-compose.yml` may be suffixed for special configurations. For example, `docker-compose.dev.yml` for development, `docker-compose.ci.yml` for testing, or `docker-compose.prod.yml` for production. `docker-compose.yml` unsuffixed should be the base/common configuration.
