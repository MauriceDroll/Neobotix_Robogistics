##############################################################################
##                                 Base Image                               ##
##############################################################################
ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO-ros-base
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN rosdep update --rosdistro $ROS_DISTRO

##############################################################################
##                                 Global Dependecies                       ##
##############################################################################
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-realsense2-description \
    ros-$ROS_DISTRO-rqt* \
    ros-$ROS_DISTRO-rviz* \
    # ros-$ROS_DISTRO-rqt-gui \
    # ros-$ROS_DISTRO-rqt-image-view \
    python3-pip \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# New versions necessary to prevent "skbuild" error from scikit-build
RUN python3 -m pip install -U pip setuptools
RUN pip3 install opencv-python

##############################################################################
##                                 Create User                              ##
##############################################################################
ARG USER=docker
ARG PASSWORD=docker
ARG UID=1000
ARG GID=1000
ARG DOMAIN_ID=1
ENV UID=$UID
ENV GID=$GID
ENV USER=$USER
ENV ROS_DOMAIN_ID=$DOMAIN_ID
RUN groupadd -g "$GID" "$USER"  && \
    useradd -m -u "$UID" -g "$GID" --shell $(which bash) "$USER" -G sudo && \
    echo "$USER:$PASSWORD" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc

USER $USER 
RUN mkdir -p /home/$USER/ros2_ws/src

##############################################################################
##                                 User Dependecies                         ##
##############################################################################
WORKDIR /home/$USER/ros2_ws/src
RUN git clone --depth 1 --branch 4.54.1 https://github.com/IntelRealSense/realsense-ros.git

WORKDIR /home/$USER/ros2_ws
RUN rosdep update --rosdistro $ROS_DISTRO
RUN rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y

##############################################################################
##                                 Build ROS and run                        ##
##############################################################################
WORKDIR /home/$USER/ros2_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install
RUN echo "source /home/$USER/ros2_ws/install/setup.bash" >> /home/$USER/.bashrc

RUN sudo sed --in-place --expression \
    '$isource "/home/$USER/ros2_ws/install/setup.bash"' \
    /ros_entrypoint.sh

CMD ["ros2", "launch", "realsense2_camera", "rs_launch.py"]
# CMD /bin/bash
