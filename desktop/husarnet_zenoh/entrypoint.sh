#!/bin/bash
#bash /limit_bandwidth.sh
#husarnet-daemon &
ROS_DISTRO=iron zenoh-bridge-ros2dds --config /advrob-desktop-dds-config.json5 --domain ${ROS_DOMAIN_ID} &
exec bash

