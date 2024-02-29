# Duke Advanced Robotics Course Project Setup

## Quick Start (Ubuntu)

### Launch operator environment

1. Launch start script with vpn joincode and advrob-user (FIRSTNAME_LASTNAME)
```
cd advrob-project
./advrob_setup.sh --advrob-user john_smith --joincode xxxxxxxxxxxxxxx
```
2. Enter the `operator` container
```
docker exec -it operator-operator-1 bash
```
3. Make sure you are able to see the relevant topics from the raspberry py
```
source /opt/ros/iron/setup.bash
ros2 topic list
```
You should see the following topics: `TODO` (for now you can verify that you are connected by running `ros2 run demo_nodes_cpp listener` and making sure you see "Hello World" messages show up every second). 

4. Any ros2 topics/services/actions that you write will pass through the following filter (`TODO`: modify filter). Make sure you only publish and subscribe to allowed topics. 
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

## Docker containers

The `advrob-project/operator/docker-compose.yaml` will start 3 docker containers. Two are for remote networking and routing, while the last one called `operator` runs `ROS2 Iron` communicates with the raspberry pi via ros2. 
Start containers:
```
cd advrob-project/operator
docker compose up -d
```
Stop containers
```
cd advrob-project/operator
docker compose down
```

## Notes
Networking
- By default operator uses ROS_DOMAIN_ID=88 and RMW_IMPLEMENTATION=rmw_fastrtps_cpp. Keep these defaults in order to be able to communicate with the pi.
