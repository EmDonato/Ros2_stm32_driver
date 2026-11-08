ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base-noble

SHELL ["/bin/bash", "-c"]

ARG ROS_DISTRO=jazzy
ARG USERNAME=stm32
ARG UID=1000
ARG GID=1000

# Noble reserves UID/GID 1000 for its default ubuntu account.
RUN if id ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi && \
    groupadd --gid ${GID} ${USERNAME} && \
    useradd --create-home --uid ${UID} --gid ${GID} --shell /bin/bash ${USERNAME} && \
    usermod --append --groups dialout,video,plugdev ${USERNAME}

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
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
