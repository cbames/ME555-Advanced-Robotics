# Duke Advanced Robotics Course Project Setup

## Quick Start

### Launch operator environment

1. Inside the operator directory: create the `.env` file based on `.env.template` and paste the Husarnet Join Code here.
2. From inside `advrob-project/operator` start the docker containers. This will start 3 contianers. Two are for remote networking and routing, while the last one called `operator` runs `ROS2 Iron` communicates with the raspberry pi via ros2. 
```
docker compose up
```
3. In another terminal enter the `operator` container.
```
docker exec -it operator-operator-1 bash
```
4. Make sure you are able to see the relevant topics from the raspberry py.
```
source /opt/ros/iron/setup.bash
ros2 topic list
```
You should see the following topics: `TODO` (for now you can verify that you are connected by looking at the terminal window where you started the containers and seeing that a "Hello World" message is being received every second). 

5. Any ros2 topics/services/actions that you write will pass through the following filter (`TODO`: modify filter). Make sure you only publish and subscribe to allowed topics. 
```
allowlist:
  - name: "ra/*"
  - name: "rp/*"
  - name: "rq/*"
  - name: "rr/*"
  - name: "rs/*"
  - name: "rt/*"
blocklist: []
builtin-topics: []
```

## Notes
Networking
- By default operator uses ROS_DOMAIN_ID=88 and RMW_IMPLEMENTATION=rmw_fastrtps_cpp. Keep these defaults in order to be able to communicate with the pi.
