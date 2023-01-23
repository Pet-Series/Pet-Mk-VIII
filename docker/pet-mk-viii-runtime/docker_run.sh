#!/bin/bash
# Run docker image
docker run --interactive \
 --rm \
 --network=host \
 --privileged \
kullken/ros2_arm64v8:pet-mk-viii-runtime