FROM arm64v8/ros:iron-ros-core

COPY ./workspace /home/workspace/

RUN apt update && apt install -y \
        ros-${ROS_DISTRO}-demo-nodes-cpp \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*
