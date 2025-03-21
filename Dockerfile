# Use the official ROS2 Humble base image
FROM ros:humble

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    git \
    && rm -rf /var/lib/apt/lists/*

COPY install/ /px4_msgs/install/

# # Create a temporary directory for building px4_msgs
# WORKDIR /px4_msgs_build/src

# # Clone px4_msgs (using the main branch; adjust if needed)
# RUN git clone https://github.com/PX4/px4_msgs.git

# # Install dependencies and build px4_msgs, installing it to /opt/ros/humble
# WORKDIR /px4_msgs_build
# RUN rosdep update \
#     && rosdep install --from-paths src --ignore-src -r -y \
#     && /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"
# WORKDIR /

# Set the entrypoint to a shell with ROS 2 sourced
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /px4_msgs/install/setup.bash && bash"]