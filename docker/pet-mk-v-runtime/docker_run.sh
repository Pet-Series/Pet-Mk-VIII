#!/bin/bash
# Run docker image
docker run --interactive \
 --rm \
 --network=host \
 --privileged \
 --volume="/home/pi/ws_ros2:/home/pet/ws_ros2" \
kullken/ros2_arm64v8:pet-mk-viii-runtime