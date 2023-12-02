#!/bin/bash

# exit immediately if a command exits with a non-zero status.
set -e

gnome-terminal --tab -e 'bash -c "roslaunch swarm_ros_bridge.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch linktrack.launch; exec bash"' \
--window -e 'bash -c "sleep 5; python ../example.py; exec bash"'
