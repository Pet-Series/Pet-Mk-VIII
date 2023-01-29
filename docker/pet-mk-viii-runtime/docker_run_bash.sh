#!/bin/bash
# Run docker image from https://hub.docker.com/repository/docker/kullken/ros2_arm64v8
# Entrypoint CMD = /bin/bash
docker run \
 -it \
 --rm \
 --network=host \
 --privileged \
 --volume="/home/pi/ws_ros2:/home/pet/ws_ros2" \
kullken/ros2_arm64v8:pet-mk-viii-runtime /bin/bash