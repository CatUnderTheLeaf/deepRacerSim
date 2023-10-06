# Build this file with
#  docker build -f deepRacerSim.Dockerfile -t deep-simulator .

ARG ROS_DISTRO=noetic
 
########################################
# Base Image for TurtleBot3 Simulation #
########################################
FROM osrf/ros:${ROS_DISTRO}-desktop-full as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]
 
# Install Python3
RUN apt-get update && apt-get install -y python3-pip


WORKDIR /deep_ws

COPY ./entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]