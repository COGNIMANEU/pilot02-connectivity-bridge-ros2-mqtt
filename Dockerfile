# Use the official ROS 2 Humble image as a base
FROM ros:humble

# Install essential dependencies for building ROS 2 packages
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Build and install Eclipse Paho MQTT C library
WORKDIR /tmp
RUN git clone https://github.com/eclipse/paho.mqtt.c.git && \
    cd paho.mqtt.c && \
    cmake -Bbuild -H. -DPAHO_WITH_SSL=ON && \
    cmake --build build/ --target install && \
    ldconfig

# Build and install Eclipse Paho MQTT C++ library
WORKDIR /tmp
RUN git clone https://github.com/eclipse/paho.mqtt.cpp.git && \
    cd paho.mqtt.cpp && \
    cmake -Bbuild -H. -DPAHO_BUILD_STATIC=OFF && \
    cmake --build build/ --target install && \
    ldconfig

# Create a workspace and copy the source code into the container
WORKDIR /workspace
COPY ./src /workspace/src

# Set environment variables
ENV LAUNCH_FILE=demo_mqtt_client_launch.py
ENV CONFIG_FILE=demo/config.yaml

# Copy config and launch folders from the host to the container (ensure these paths exist on the host)
COPY ./config /workspace/src/mqtt_client/config
COPY ./launch /workspace/src/mqtt_client/launch

# Build the workspace using colcon
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Source the workspace setup script by default
RUN echo 'source /workspace/install/setup.bash' >> /root/.bashrc

# Default command to source the workspace and run a specified launch file
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch mqtt_client $LAUNCH_FILE"]