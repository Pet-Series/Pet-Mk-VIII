#!/bin/bash
# Run docker image from https://hub.docker.com/repository/docker/kullken/ros2_arm64v8
docker run \
 -it \
 --rm \
 --network=host \
 --privileged \
 kullken/ros2_arm64v8:pet-base-humble