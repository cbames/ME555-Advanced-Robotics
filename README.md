# Duke Advanced Robotics Course Project Setup

## Quick Start

### Launch operator environment

1. Inside the operator directory: create the `.env` file based on `.env.template` and paste the Husarnet Join Code here.
2. Start the docker containers. This will start 3 contianers. Two are for remote networking and routing, while the last one called `operator` runs `ROS2 Iron` and is is able to communicate with the raspberry pi
via ros2. 
```
docker compose up
```
3. Enter the `operator` container.
```
docker exec -it operator-operator-1 bash
```
4. Make sure you are able to see the relevant topics from the raspberry py.
```
/opt/ros/iron/setup.bash
ros2 topic list
```
You should see the follwoing topics: `TODO`

## Notes
Networking
- By default operator uses ROS_DOMAIN_ID=88 and RMW_IMPLEMENTATION=rmw_fastrtps_cpp. Keep these defaults in order to be able to communicate with the pi.
