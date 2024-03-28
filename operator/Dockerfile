FROM ros:iron-ros-core

ENV ROS_DISTRO=iron
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*


# install ros2 packages
RUN apt update && apt install -y \
        ros-$ROS_DISTRO-demo-nodes-cpp \
	ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
	ros-$ROS_DISTRO-rviz2 \
	ros-$ROS_DISTRO-image-transport-plugins

# setup entrypoint
COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
