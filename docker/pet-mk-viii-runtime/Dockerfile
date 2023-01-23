# docker build -t pet-mk-viii .
#
# Launch image with:
#----------------------
# docker run -it \
#  --rm \
#  --network=host \
#  --privileged \
# pet-mk-viii
#----------------------
FROM  kullken/ros2_arm64v8:pet-base-humble 
ENV DOCKERUSER="pet"
ENV HOMEDIR="/home/${DOCKERUSER}"
ENV DOCKERPROMPT="Pet-Mk.VIII"

LABEL description="ROS2 Humble image for Pet-Mk.VIII - Launch all basic ROS2 nodes"
LABEL maintainer="Stefan Kull <stefan.kull@gmail.com>"
LABEL version="1.0"

SHELL ["/bin/bash", "-c"]


RUN echo 'export PS1="${debian_chroot:+($debian_chroot)}\[\033[01;37;44m\]🐳${DOCKERPROMPT}\[\033[00m\]_\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\\$ "' >> /home/pet/.bashrc 

# Create ROS2 work space
USER ${DOCKERUSER}
RUN mkdir --parent ${HOMEDIR}/ws_ros2/src
WORKDIR ${HOMEDIR}/ws_ros2/src

RUN git config --global user.email "stefan.kull@gmail.com"
RUN git config --global user.name "SeniorKullken"
RUN git clone https://github.com/Pet-Series/Pet-Mk-VIII.git 

WORKDIR ${HOMEDIR}/ws_ros2
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install
RUN echo 'source ${HOMEDIR}/ws_ros2/install/setup.bash' >> /home/pet/.bashrc 

# Bring up ROS2 nodes
WORKDIR ${HOMEDIR}/ws_ros2
RUN mkdir --parent ./src/Pet-Mk-VIII/docker/pet-mk-viii-runtime
COPY ./entrypoint.sh  ./src/Pet-Mk-VIII/docker/pet-mk-viii-runtime/entrypoint.sh
USER root
RUN chmod +x ./src/Pet-Mk-VIII/docker/pet-mk-viii-runtime/entrypoint.sh
USER ${DOCKERUSER}

ENTRYPOINT ["/home/pet/ws_ros2/src/Pet-Mk-VIII/docker/pet-mk-viii-runtime/entrypoint.sh"]
# launch ros package
# CMD ["ros2", "launch", "pet_mk_viii", "pet-mk-viii-bribgup.py"]