# =========================
# Base image (ROS2 Humble)
# =========================
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# =========================
# System dependencies
# =========================
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    nano \
    vim \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# =========================
# ROS2 dependencies
# =========================
RUN apt-get update && apt-get install -y \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-pcl-ros \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-vision-opencv \
    && rm -rf /var/lib/apt/lists/*

# =========================
# Math / mapping libs
# =========================
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libpcl-dev \
    libomp-dev \
    && rm -rf /var/lib/apt/lists/*

# =========================
# Initialize rosdep
# =========================
RUN rosdep init || true && rosdep update

# =========================
# Workspace
# =========================
WORKDIR /workspace
RUN bash -c "mkdir /workspace/src/"
#RUN bash -c "mkdir /workspace/src/falcon_mapping_node"


# =========================
# Copy your package
# =========================
# (assumes Docker build context contains your ROS2 package)
#COPY ./src /workspace/src/
#COPY src/falcon_mapping_node/include /workspace/src/falcon_mapping_node/
#COPY src/falcon_mapping_node/launch /workspace/src/falcon_mapping_node/
#COPY ./CMakeLists.txt /workspace/src/falcon_mapping_node/
#COPY ./package.xml /workspace/src/falcon_mapping_node/



# =========================
# Build workspace
# =========================
#RUN . /opt/ros/humble/setup.sh && \
#    rosdep install --from-paths src --ignore-src -r -y && \
#    colcon build --symlink-install

# =========================
# Environment setup
# =========================
#RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
#    echo "source /workspace/install/setup.bash" >> ~/.bashrc

# =========================
# Default command
# =========================
CMD ["/bin/bash"]