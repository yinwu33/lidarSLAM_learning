ROS_DISTRO="kinetic"

# set up sources.list
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# set up keys
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

apt-get update
apt-get install -y ros-$ROS_DISTRO-desktop-full

# environment
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/ubuntu/.bashrc
source /home/ubuntu/.bashrc

apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# rosdep
su ubuntu
rosdep init
rosdep update
