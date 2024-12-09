# Use the official ROS Humble base image
FROM ros:humble-perception

# Install common dependencies
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    libboost-all-dev \
    libopus-dev \
    vim \
    net-tools \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Add ROS setup to .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

# Set the default workspace
WORKDIR /ros2_ws

# Set the entrypoint to bash
ENTRYPOINT ["/bin/bash"]
CMD ["bash"]