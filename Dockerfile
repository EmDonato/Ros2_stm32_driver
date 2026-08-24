ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base-jammy

SHELL ["/bin/bash", "-c"]

ARG ROS_DISTRO=humble
ARG USERNAME=stm32
ARG UID=1000
ARG GID=1000

RUN groupadd --gid ${GID} ${USERNAME} && \
    useradd --create-home --uid ${UID} --gid ${GID} --shell /bin/bash ${USERNAME} && \
    usermod --append --groups dialout,video,plugdev ${USERNAME}

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    libboost-dev \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY --from=interfaces /src/robot_interfaces src/robot_interfaces/
COPY src/ src/
COPY config/ config/

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker-entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && \
    chown -R ${USERNAME}:${USERNAME} /ws

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

USER ${USERNAME}

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "stm32_driver", "stm32_driver_node", "--ros-args", "--params-file", "/ws/config/params.yaml"]
