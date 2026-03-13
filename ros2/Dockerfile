FROM ros:humble

# Install Python build deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws

# Copy shared dashboard modules (protocol.py, car_config.py)
COPY dashboard/protocol.py  /tmp/dashboard/protocol.py
COPY dashboard/car_config.py /tmp/dashboard/car_config.py

# Copy ROS2 package
COPY ros2/src/umbreon_bridge/ src/umbreon_bridge/

# Install shared modules into the package, patching imports
RUN cp /tmp/dashboard/protocol.py  src/umbreon_bridge/umbreon_bridge/protocol.py && \
    cp /tmp/dashboard/car_config.py src/umbreon_bridge/umbreon_bridge/car_config.py && \
    sed -i 's/^from car_config /from umbreon_bridge.car_config /' \
        src/umbreon_bridge/umbreon_bridge/protocol.py

# Build
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select umbreon_bridge

# Source workspace on entry
RUN echo '. /opt/ros/humble/setup.sh' >> /ros_entrypoint.sh && \
    echo '. /ros2_ws/install/setup.sh' >> /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "run", "umbreon_bridge", "bridge_node"]
