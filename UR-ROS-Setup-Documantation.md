### initial Situation
- older UR Polyscope Software which is currently out of support
- ROS setup with ROS-foxy
- ROS interface for neobotix-coordinator (behavior-tree with ottdated iras-interfaces & petra-interfaces packages) - also in ROS-foxy

### What has been done so far:
- update UR Polyscope on the teachpad to version  **XXXXXXXXXXXXXXXXXXXXXXXXXXXX**
- setup of a completely new docker container to run ROS-humble (ur5-driver-humble) --> Dockerfile , build.sh and start.sh
  - install ur packages with apt inside the dockerfile
- implementation of the common iras python interface (as known from the r2e cells)
    - required depenend packages copied into the ```docker/ws_dependencies``` --> moveit_wrapper, ros_enviroment, py_dependencies with additional reference classes for affine-transform calculations
    - extend Dockerfile to build and source the dependencies proper
    - write new launchfile in moveit_wrapper package to launch moveit-nodes and the wrapper-node (providing new service servers) --> only the minimum of required parameters is implemented as launch arguments (for simplicity reasons)
- setup of the robot_application package to run the python applications --> ```docker/ros2_ws/src/robot_application``` is mounted to the host machine!
    - own python scripts can be edited local on the host machine in ```ur-driver-humble/src/robot_application/programs```

### Usage:
1) start the docker container (ur.driver.humble) with ```./start.sh```
2) build the ros2_ws (in specific the robot_application package) with ```colcon build``` 
3) source the ros2_ws with ```source install/setup.bash``` (note: the dependencies are built and sourced automatically while building the container)
4) start the drivers: ```ros2 launch ur_robot_driver ur_type:=ur5 robot_ip:=192.168.1.103 launch_rviz:=false```
5) start the remote control (program = "external_control") on the UR teachpanel and put the speed-slider bar to 100% because the trajectory execution in ros is calculated on this base and speed reduction isnt possible
6) connect anonther terminal to the container: ```docker exec -it ur-driver-humble bash``` and source it
7) launch moveit and the wrapper: ```ros2 launch moveit_wrapper wrapper_complete.launch.py ur_type:=ur5``` (note: launch_rviz:=true by default)
8) connect another terminal to the container --> 5)
9) run the python script you have written your application in: ```ros2 run robot_application <program-name>```

--> for more informations on how to write python applications please refer here: [application-tutorial for diy-robotics](https://github.com/RobinWolf/diy_robot_application/tree/main)

### ToDo:
- modify robot_description URDF with neobotix, camera-mount, camera and gripper --> self-collision checking, URDF package is implemented as launch argument!
- maybe also modify the SRDF file?


