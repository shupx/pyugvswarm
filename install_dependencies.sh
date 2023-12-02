#!/bin/sh

# Exit immediately if a command exits with a non-zero status.
set -e


# Install vrpn  # change 'noetic' into 'melodic' for ubuntu18:
sudo apt install -y ros-noetic-vrpn-client-ros


# If you use UWB for positioning, then you should build `nlink_parser` (https://www.nooploop.com/download) ROS package for nooploop UWB module:
cd ~
mkdir -p nlink_parser_ws/src
cd nlink_parser_ws/src
git clone --recursive https://gitee.com/shu-peixuan/nlink_parser.git
cd ../
catkin_make
catkin_make
echo "source ~/nlink_parser_ws/devel/setup.bash" >> ~/.bashrc
