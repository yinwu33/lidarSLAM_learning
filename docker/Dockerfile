FROM fredblgr/ubuntu-novnc:20.04

LABEL maintainer="YinWu<yinwu.life@gmail.com>"

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8 HOME=/root APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    #
    sudo pkg-config gpg-agent build-essential lsb-release\
    # networks
    curl wget net-tools iputils-ping\
    # c++
    gcc g++ cmake \
    # version control 
    git \
    # editor
    vim \
    # development
    libopencv-dev libeigen3-dev 

# ------ set up ROS ------
COPY packages/ros_install.sh /root/init/
RUN chmod +x /root/init/ros_install.sh && /root/init/ros_install.sh

EXPOSE 6080