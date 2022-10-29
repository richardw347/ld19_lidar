FROM ros:rolling-ros-base

ENV ROS2_WS /opt/ros2_ws
RUN mkdir -p $ROS2_WS/src
WORKDIR $ROS2_WS

RUN mkdir -p $ROS2_WS/src/ld19_lidar
COPY . $ROS2_WS/src/ld19_lidar/

RUN apt-get update && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# setup entrypoint
COPY ./ros_entrypoint.sh /opt

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
    --cmake-args \
    -DSECURITY=ON --no-warn-unused-cli \
    --symlink-install

RUN ["chmod", "+x", "/opt/ros_entrypoint.sh"]

CMD ["/opt/ros_entrypoint.sh"]