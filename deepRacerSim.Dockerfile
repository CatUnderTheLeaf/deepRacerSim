# Build this file with
#  docker build -f deepRacerSim.Dockerfile -t deep-simulator .

ARG ROS_DISTRO=noetic
 
########################################
# Base Image for DeepRacer Simulation #
########################################
FROM osrf/ros:${ROS_DISTRO}-desktop-full as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]
 
# Install Git and Python3
RUN apt-get update && apt-get install -y git python3-pip

# Download this repository
RUN git clone https://github.com/CatUnderTheLeaf/deepRacerSim.git
# TODO instead of clone replace with volume

# Set the working folder at startup
WORKDIR /deepRacerSim/deep_ws

# Install dependencies
RUN rm -r /etc/ros/rosdep/sources.list.d/20-default.list
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN yes | rosdep install --from-paths src --ignore-src --rosdistro noetic

# Build the Catkin workspace and ensure it's sourced
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3'
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /deepRacerSim/deep_ws/devel/setup.bash" >> ~/.bashrc
 
# export Gazebo variables
RUN echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/deepRacerSim/deep_ws/src/simulation/tracks/" >> ~/.bashrc
RUN echo "export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/deepRacerSim/deep_ws/src/simulation/" >> ~/.bashrc
