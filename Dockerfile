ARG ROS=melodic
FROM ros:${ROS}-ros-base

SHELL ["/bin/bash", "-xvc"]

RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-tf* \
    python-catkin-tools

WORKDIR /root

# ROS setting
RUN /bin/bash -c "mkdir -p catkin_ws/src"

RUN cd catkin_ws && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin build"

RUN cd /root && echo source /root/catkin_ws/devel/setup.bash >> .bashrc

ENV ROS_PACKAGE_PATH=/root/catkin_ws:$ROS_PACKAGE_PATH

ENV ROS_WORKSPACE=/root/catkin_ws

RUN ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen

RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root
