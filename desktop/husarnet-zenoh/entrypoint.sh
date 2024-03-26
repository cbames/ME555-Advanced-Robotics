#!/bin/bash
#bash /limit_bandwidth.sh
husarnet-daemon &
ROS_DISTRO=humble zenoh-bridge-ros2dds --connect tcp/operator:7447 --config /wall-panels-team-dds-config.json5 --domain ${ROS_DOMAIN_ID} &
exec bash

