# Check docs.ros.org for the current LTS release before relying on this tag.
FROM ros:jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-serial python3-pytest socat build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . src/serial_imu_bridge
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

CMD ["bash", "-c", ". /opt/ros/jazzy/setup.sh && . install/setup.sh && ros2 launch serial_imu_bridge bridge.launch.py"]
