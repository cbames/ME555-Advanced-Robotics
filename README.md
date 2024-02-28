# Duke Advanced Robotics Course Project Setup

## Quick Start

### Launch operator environment

1. Inside the operator directory: create the `.env` file based on `.env.template` and paste the Husarnet Join Code here.
2. From inside `advrob-project` start the docker containers. This will start 3 contianers. Two are for remote networking and routing, while the last one called `operator` runs `ROS2 Iron` and is is able to communicate with the raspberry pi
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
source /opt/ros/iron/setup.bash
ros2 topic list
```
You should see the follwoing topics: `TODO`

5. Create a directory with your first and last name in the `/home/workspace/` directory in the following format: `FIRSTNAME_LASTNAME` (e.g. `JOHN_SMITH`). Make sure to work and save anything you want to persist in the `/home/workspace/` directory. Only files in this directory will be saved. 
```
mkdir FIRSTNAME_LASTNAME
```

## Notes
Networking
- By default operator uses ROS_DOMAIN_ID=88 and RMW_IMPLEMENTATION=rmw_fastrtps_cpp. Keep these defaults in order to be able to communicate with the pi.
