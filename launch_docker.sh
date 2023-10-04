#!/bin/bash
# launch a Docker container
#
# Usage Example:
# ./launch_docker.sh 

# Define Docker volumes and environment variables
# DOCKER_VOLUMES="
# --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
# --volume="${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority" \
# "
DOCKER_ENV_VARS="
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="NVIDIA_DRIVER_CAPABILITIES=all" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
"
# DOCKER_ARGS=${DOCKER_VOLUMES}" "${DOCKER_ENV_VARS}
DOCKER_ARGS=${DOCKER_ENV_VARS}

# Run the command
docker run -it --net=host --gpus all --ipc=host --privileged ${DOCKER_ARGS} deep-simulator bash

# Run the command with params (if added)
# ./launch_docker.sh deep-simulator "roslaunch simulation simulation.launch"
# docker run -it --net=host --ipc=host --privileged ${DOCKER_ARGS} "$1" bash -c "$2"