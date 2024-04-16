# Neobotix_Robogistics
## Commands
ros2 launch neo_mpo_500-2 mapping.launch.py

https://neobotix-docs.de/ros/ros2/autonomous_navigation.html




## Dockerfiles
### Neobotic_mmo500_driver
##############################################################################
##                                 User Dependecies                         ##
##############################################################################
WORKDIR /home/$USER/ros2_ws/src
RUN git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_mpo_500-2.git
RUN git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_nav2_bringup.git
RUN git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_local_planner2.git
RUN git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_localization2.git
RUN git clone --branch master          https://github.com/neobotix/neo_common2
RUN git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_relayboard_v2-2
RUN git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_kinematics_mecanum2.git
RUN git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_sick_s300-2
RUN git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_teleop2
RUN git clone --branch master          https://github.com/neobotix/neo_msgs2
RUN git clone --branch master          https://github.com/neobotix/neo_srvs2 

### Neobotix_coordinator
##############################################################################
##                                 User Dependecies                         ##
##############################################################################
WORKDIR /home/$USER/ros2_ws/src
RUN git config --global advice.detachedHead false
RUN git clone --depth 1 -b v1.3.0 https://project_55_bot:glpat-DjsyN_ixYnq-duDb_Sip@www.w.hs-karlsruhe.de/gitlab/iras/research-projects/petra/petra_interfaces.git
RUN git clone --depth 1 -b v1.0.0 https://project_240_bot:glpat-stK1tgiDxr44VV8X95r7@www.w.hs-karlsruhe.de/gitlab/iras/common/behaviortree_ros.git
RUN git clone --depth 1 -b v1.0.0 https://project_109_bot:glpat-ydJpMPNbrfjXxzYoJbvS@www.w.hs-karlsruhe.de/gitlab/iras/core/cpp_core.git
RUN git clone --depth 1 -b v1.0.0 https://project_146_bot:glpat-QfHya4tsosmRTkDgkymW@www.w.hs-karlsruhe.de/gitlab/iras/research-projects/petra/behavior-tree/petra_communication.git
# RUN git clone --depth 1 https://github.com/BehaviorTree/Groot.git
RUN git clone --depth 1 -b ros2 https://github.com/AndreasZachariae/audio_common.git
COPY . ./neobotix_coordinator
COPY ./sounds ./audio_common/sound_play/sounds
