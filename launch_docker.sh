#!/bin/bash
# launch a Docker container
#
# Usage Example:
# ./launch_docker.sh 

DOCKER_ENV_VARS="
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="NVIDIA_DRIVER_CAPABILITIES=all" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
"
# if use repository as volume
# --volume="${PWD}/deep_ws/src":"/deep_ws/src":rw \
# DOCKER_ARGS=${DOCKER_VOLUMES}" "${DOCKER_ENV_VARS}
DOCKER_ARGS=${DOCKER_ENV_VARS}

# Run the command
docker run -it --net=host --gpus all --ipc=host --privileged ${DOCKER_ARGS} deep-simulator bash

# Run the command with params (if added)
# ./launch_docker.sh deep-simulator "roslaunch simulation simulation.launch"
# docker run -it --net=host --ipc=host --privileged ${DOCKER_ARGS} "$1" bash -c "$2"