FROM ros:iron-ros-core

ENV ROS_DISTRO=iron
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool 


# install ros2 packages
RUN apt-get update && apt-get install --no-install-recommends -y \
        ros-$ROS_DISTRO-demo-nodes-cpp \
	ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
	ros-$ROS_DISTRO-rviz2 \
	ros-$ROS_DISTRO-image-transport-plugins

# install zenoh
RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | tee -a /etc/apt/sources.list > /dev/null && \
    apt-get update -y && \
    apt-get install -y zenoh-bridge-ros2dds || true

# setup entrypoint
COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
