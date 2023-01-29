#!/bin/bash
# Run docker image from https://hub.docker.com/repository/docker/kullken/ros2_arm64v8
# Default ENTRYPOINT 'CMD ["ros2", "launch", "pet_mk_viii", "pet-mk-viii-bringup.launch.py"]'
docker run \
 -it \
 --rm \
 --network=host \
 --privileged \
 --volume="/home/pi/ws_ros2:/home/pet/ws_ros2" \
kullken/ros2_arm64v8:pet-mk-viii-runtime