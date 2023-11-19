# ros2_esp32car
RC Car with micro-ROS and ESP32 

https://github.com/shirokunet/ros2_esp32car/assets/36523448/f5150fb2-e22d-48d4-ae6e-6ca5d139a4a1

## Dependency
- ROS2 Humble
- micro_ros_arduino v2.0.7-humble

## Prepare Hardware
### Build a your own bot
![PXL_20231119_142233976](https://github.com/shirokunet/ros2_esp32car/assets/36523448/72c72169-ab54-4748-a9e3-99f5716489ad)

![PXL_20231119_143026900](https://github.com/shirokunet/ros2_esp32car/assets/36523448/113d2118-dc7a-4b4e-a440-f3889d6b4d7c)

I bought below parts.
- ¥1,600 [ESP32-DevKitC-32E](https://akizukidenshi.com/catalog/g/gM-15673/)
- ¥550 [DRV8835](https://akizukidenshi.com/catalog/g/gK-09848/)

## Getting started
Make sure you need an access point.
```
$ git clone https://github.com/shirokunet/ros2_esp32car
$ cd ros2_esp32car/
$ cp include/keys_example.h include/keys.h
$ vi keys.h
char wifi_ssid[20] = "WIFI_SSID";
char wifi_pass[20] = "WIFI_PASS";
char agent_ip[20] = "192.168.128.111";
unsigned int agent_port = 8888;
```

## Build and Upload
```
$ pio run -e main
$ pio run -e main --target nobuild --target upload --upload-port /dev/ttyUSB0
```

## Run ROS2 nodes for sending commands
### Run micro-ROS agent
```
$ ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### Run joy_node
```
$ ros2 run joy joy_node
```

### Run teleop_node
```
$ ros2 launch teleop_twist_joy teleop-launch.py joy_config:='f310'
```

### Useful commands for debugging
```
$ ros2 topic list
$ ros2 topic echo /joy
$ ros2 topic echo /cmd_vel
```

## teleop_twist_joy for Logicool Gamepad F310
If you do not have a PS4/5 controller, you can use a F310 with my repository.
- [shirokunet/teleop_twist_joy](https://github.com/shirokunet/teleop_twist_joy)
