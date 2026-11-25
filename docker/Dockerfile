FROM osrf/ros:jazzy-desktop-full

# Setup user for GUI to work without xhost
ARG UNAME=user
ARG UID=1000
ARG GID=1000
RUN groupadd -f render \
    && groupadd -g ${GID} -o ${UNAME} \
    && useradd -m -u ${UID} -g ${GID} -o -s /bin/bash ${UNAME} \
    && usermod -aG sudo,video,render ${UNAME}
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER ${UNAME}

# Create a custom ros2 overlay workspace and workspace for exercises
ENV ROS2_WS=/home/user/ros2_ws
RUN mkdir -p ${ROS2_WS}/src
ENV EXERCISES_WS=/home/user/exercises_ws
RUN mkdir -p ${EXERCISES_WS}/src

# Disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive

# Make default shell in Dockerfile bash instead of sh
SHELL ["/bin/bash", "-c"]

# Copy requirement files and install ROS 2 Jazzy & Gazebo Harmonic dependencies
COPY docker/andino_requirements.txt .
RUN sudo apt-get update && sudo apt-get install --no-install-recommends -y $(cat andino_requirements.txt) && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/* && \
    sudo rm andino_requirements.txt

# Copy course packages, clone Andino repositories, apply Jazzy patches, and build ros2_ws
COPY --chown=${UNAME}:${UNAME} ./packages/ ${ROS2_WS}/src/
COPY ./docker/patches/ /tmp/patches/

RUN cd ${ROS2_WS}/src && \
    git clone https://github.com/Ekumen-OS/andino.git && \
    git -C andino checkout 974264b && \
    touch andino/andino_base/COLCON_IGNORE \
          andino/andino_bringup/COLCON_IGNORE \
          andino/andino_firmware/COLCON_IGNORE \
          andino/andino_hardware/COLCON_IGNORE && \
    git clone -b jazzy https://github.com/Ekumen-OS/andino_gz.git && \
    git -C andino_gz checkout 073dc67 && \
    git -C andino_gz apply /tmp/patches/andino_gz.patch && \
    sudo rm -rf /tmp/patches && \
    sudo apt-get update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${ROS2_WS} && \
    rosdep update && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/* && \
    colcon build --symlink-install

# Build exercises_ws
RUN cd ${EXERCISES_WS} && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source ${ROS2_WS}/install/setup.bash && \
    colcon build --symlink-install

# Source ROS workspaces automatically when new terminal is opened
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${ROS2_WS}/install/setup.bash" >> ~/.bashrc && \
    echo "source ${EXERCISES_WS}/install/setup.bash" >> ~/.bashrc && \
    echo "export QT_QPA_PLATFORM=xcb" >> ~/.bashrc

WORKDIR /home/user/exercises_ws

# Source ROS in the main terminal
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
