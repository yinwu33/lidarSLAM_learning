FROM dorowu/ubuntu-desktop-lxde-vnc:xenial
LABEL maintainer="Tiryoh<tiryoh@gmail.com>"

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/sh", "-c"]

RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq \
    # dependencies
    apt-utils sudo lsb-release locales bash-completion tzdata gosu \
    # network
    wget curl net-tools \
    # development
    git build-essential vim  pkg-config \
    # c++
    gcc g++ cmake && \
    rm -rf /var/lib/apt/lists/*

RUN useradd --create-home --home-dir /home/ubuntu --shell /bin/bash --user-group --groups adm,sudo ubuntu && \
    echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ------ set up ROS ------
COPY ros_install.sh /home/ubuntu/init/
RUN chmod +x /home/ubuntu/init/ros_install.sh && /home/ubuntu/init/ros_install.sh
ENV USER ubuntu

# ------ for lidar slam lesson ------
RUN apt install -y ros-kinetic-csm ros-kinetic-nav-core
