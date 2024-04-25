# RealSense Driver for ROS2

This is the ROS2 driver for the RealSense D400 and L500 series SR300 camera and T265 Tracking Module.

It is running in a docker container and based on the official [realsense-ros from IntelRealSense](https://github.com/IntelRealSense/realsense-ros/tree/ros2).

## How to run

### Linux
Simply build and run the docker container with:
```bash
source start_docker.sh
```

Inside the docker launches a default driver
```bash
ros2 launch realsense2_camera rs_launch.py
```

### MacOS
Set `uid=1000` and `gid=1000` in the `start_docker.sh`.
Then build and run the docker container with:
```bash
source start_docker.sh
```

## Troubleshooting

### Error: failed to open usb interface: 0

https://github.com/IntelRealSense/realsense-ros/issues/1408

Copy the file `99-realsense-libusb.rules` on the host system into `/etc/udev/rules.d/`