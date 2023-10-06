#!/bin/bash

# Source ROS and Catkin workspaces
source /opt/ros/noetic/setup.bash

cd /deep_ws

# Install dependencies
rm -r /etc/ros/rosdep/sources.list.d/20-default.list
rosdep init \
 && rosdep fix-permissions \
 && rosdep update
yes | rosdep install --from-paths src --ignore-src --rosdistro noetic

# Build the Catkin workspace and ensure it's sourced
/bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3'
source /deep_ws/devel/setup.bash
 
# export Gazebo variables
# RUN echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/deepRacerSim/deep_ws/src/simulation/tracks/" >> ~/.bashrc
# RUN echo "export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/deepRacerSim/deep_ws/src/simulation/" >> ~/.bashrc
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/deep_ws/src/simulation/tracks/
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/deep_ws/src/simulation/

# Execute the command passed into this entrypoint
exec "$@"