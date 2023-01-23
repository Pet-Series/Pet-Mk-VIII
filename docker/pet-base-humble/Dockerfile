# Base Docker Image
# 1) ROS Humble
# 2) Create Docker user "pet" with access to USB, I2C and GPIO
# 3) Add "apt" package
# 4) Add "pip3" package

# Docker Build image with:
#----------------------
# docker build -t pet-base-humble .
#----------------------
#
# Docker Launch image with:
#----------------------
# $ docker run -it --rm --network=host --privileged pet-base-humble
#----------------------

# https://hub.docker.com/r/arm64v8/ros/tags
# https://hub.docker.com/r/arm64v8/ros/tags?page=1&name=humble
FROM arm64v8/ros:humble-ros-base-jammy
ENV DOCKERUSER="pet"
ENV HOMEDIR="/home/${DOCKERUSER}"
ENV DOCKERPROMPT="Pet-Base-Humble"

LABEL description="ROS2 Humble image for RasperryPI with I2C & GPIO access"
LABEL maintainer="Stefan Kull <stefan.kull@gmail.com>"
LABEL version="1.0"

SHELL ["/bin/bash", "-c"]

# Setup Docker user
RUN groupadd --force --gid 1001 gpio
RUN groupadd --force --gid 139 i2c
RUN groupadd --force --gid 20 dialout
RUN groupadd --force --gid 27 sudo
RUN useradd --uid 1000 --shell /bin/bash --home-dir ${HOMEDIR} --create-home --user-group --groups gpio,i2c,dialout,sudo ${DOCKERUSER}
RUN echo "${DOCKERUSER}:d0cker" | chpasswd
USER ${DOCKERUSER}
WORKDIR ${HOMEDIR}

RUN echo 'export PS1="${debian_chroot:+($debian_chroot)}\[\033[01;37;44m\]🐳${DOCKERPROMPT}\[\033[00m\]_\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\\$ "' >> /home/pet/.bashrc 

# Install required software i2c
USER root
RUN apt update -y && apt -y install i2c-tools python3-pip

# Install Neofetch
USER root
RUN sudo apt update -y && apt -y install neofetch && echo 'neofetch' >> /home/pet/.bashrc 

# Install Python3 extentions for I2C, GPIO and AD-converter
USER ${DOCKERUSER}
RUN pip3 install RPi.GPIO Adafruit_ADS1x15 board adafruit-blinka lgpio
# WARNING: The scripts pyserial-miniterm and pyserial-ports are installed in '/home/pet/.local/bin' which is not on PATH.

# Install VisualCode
# USER root
# RUN apt update -y && apt -y install software-properties-common apt-transport-https wget
#RUN wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
#RUN wget -O- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor | tee /usr/share/keyrings/vscode.gpg
#RUN add-apt-repository "deb [arch=arm64] https://packages.microsoft.com/repos/vscode stable main"
#RUN echo deb [arch=arm64 signed-by=/usr/share/keyrings/vscode.gpg] https://packages.microsoft.com/repos/vscode stable main | tee /etc/apt/sources.list.d/vscode.list
#RUN apt update -y && apt install -y code
USER ${DOCKERUSER}