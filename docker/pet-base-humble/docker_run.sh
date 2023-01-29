#!/bin/bash
# Run docker image
#  --entrypoint "/bin/bash" \
docker run \
 -it \
 --rm \
 --network=host \
 --privileged \
 kullken/ros2_arm64v8:pet-base-humble