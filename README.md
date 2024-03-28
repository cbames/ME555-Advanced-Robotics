# Duke Advanced Robotics Course Project Setup

## Quick Start (Ubuntu)

### Launch operator environment

1. Launch start script with the vpn joincode and advrob-user (FIRSTNAME_LASTNAME)
```
cd advrob-project/operator
./advrob_setup.sh -j xxxxxxxxxxxxxxx
```
2. Enter the `operator` container
```
docker exec -it operator bash
```
3. Make sure you are able to see the relevant topics from the lab computer
```
source /opt/ros/iron/setup.bash
ros2 topic list
```
You should see the following topics: `TODO` (for now you can verify that you are connected by running `ros2 run demo_nodes_cpp listener` and making sure you see "Hello World" messages show up every second). 

4. Any ros2 topics/services/actions that you write will pass through the following filter (`TODO`: modify filter). Make sure you only publish and subscribe to allowed topics. 
```
allow: {
   publishers: ["*"],
   subscribers: ["*"],
   service_servers: ["*"],
   service_clients: ["*"],
   action_servers: ["*"],
   action_clients: ["*"],
}
```

## Notes
### Server setup
Setting up the server for the first time
```
cd advrob-project/desktop
./desktop_setup.sh -j xxxxxxxxxxxxxxx
```
Running RealSense camera topics
```
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=680x480x4 pointcloud.enable:=true config_file:=./desktop/realsense_ffmpeg_config.yaml
```


### Networking
- By default operator uses ROS_DOMAIN_ID=0 and RMW_IMPLEMENTATION=rmw_cyclonedds_cpp. Keep these defaults in order to be able to communicate with the control computer.
### Docker
The `advrob-project/operator/compose.yaml` will start 4 docker containers. 3 are for remote networking and routing, while the last one called `operator` runs `ROS2 Iron` communicates with the controller computer via ros2. 
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
