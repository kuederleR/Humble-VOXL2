# Use the official ROS2 Humble base image
FROM ros:humble

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    && rm -rf /var/lib/apt/lists/*

COPY /data/voxl-suite-offline-packages/voxl-mpa-to-ros2_*.deb /tmp/
RUN dpkg -i /tmp/voxl-mpa-to-ros2_*.deb || apt-get install -f -y \
    && rm /tmp/voxl-mpa-to-ros2_*.deb

# Copy the files in the subdirectories to the container
# COPY voxl-mpa-to-ros2/ /voxl-mpa-to-ros2/

# # Initialize rosdep
# RUN rosdep init && rosdep update

# # Create a workspace
# RUN mkdir -p /ros2_ws/src

# # Set the working directory
# WORKDIR /ros2_ws

# # Copy the package files
# COPY . /ros2_ws

# # Install dependencies
# RUN rosdep install --from-paths src --ignore-src -r -y

# # Build the workspace
# RUN colcon build

# # Source the setup script
# RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# # Set the entrypoint
# ENTRYPOINT ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && bash"]