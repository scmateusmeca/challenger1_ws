#!/bin/bash

source /home/bir/Documents/workspaces_ros/Projetos/challengeBomb_ws/devel/setup.bash
roslaunch cimatec_simulation test_camera.launch &
python2 /home/bir/Documents/workspaces_ros/Projetos/challengeBomb_ws/src/cimatec_simulation/scripts/objectDetection.py