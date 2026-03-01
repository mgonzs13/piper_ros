ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO} AS deps

SHELL ["/bin/bash", "-c"]

# Copy sources and import dependencies via vcs
WORKDIR /root/ros2_ws
COPY . src/piper_ros/
RUN apt-get update && \
    apt-get install -y python3-vcstool && \
    vcs import src < src/piper_ros/dependencies.repos && \
    rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
RUN apt-get update && \
    rosdep update --include-eol-distros && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace with colcon
FROM deps AS builder
ARG CMAKE_BUILD_TYPE=Release

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}

# Source the workspace on login
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
