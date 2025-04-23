FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime

# Install necessary packages
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    python3 \
    python3-pip \
    python3-venv \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Add the ROS2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
ros-humble-desktop \
python3-colcon-common-extensions \
git \
&& rm -rf /var/lib/apt/lists/*

# Setup ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /app