# ros2-workspace-template

Template for ROS2 ([Humble Hawksbill](https://docs.ros.org/en/humble/)) workspace using [VSCode Remote Container](https://code.visualstudio.com/docs/remote/containers) with [Docker Compose](https://docs.docker.com/compose/).

## Conventions

### `Dockerfile` Naming

`Dockerfile` may be suffixed when multiple images (with different dependencies) are built within a project. For example, `Dockerfile.ui` for the production UI image, `Dockerfile.talker` for the ROS2 talker node, and `Dockerfile.listener` for the ROS2 listener node. If there is only one image or a "main image", `Dockerfile` should be unsuffixed.

For development, ideally a single `Dockerfile.dev` that is comprehensive (i.e. having all dependencies) should be used (to minimize rebuilding & wasted space for multiple images). To test multiple ROS2 nodes, it is better to just change the command used in `docker-compose.dev.yml` (for example, `/bin/sh -c "source install/local_setup.sh && ros2 run pubsub talker").

### `docker-compose.yml` Naming

`docker-compose.yml` may be suffixed for special configurations. For example, `docker-compose.dev.yml` for development, `docker-compose.ci.yml` for testing, or `docker-compose.prod.yml` for production. `docker-compose.yml` unsuffixed should be the base/common configuration.
