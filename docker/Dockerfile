# RBY1 ros2_control driver image (build + runtime).
#
#   docker build -f docker/Dockerfile --build-arg ROS_DISTRO=humble -t rby1-ros2:humble .
#   docker buildx build --platform linux/arm64 -f docker/Dockerfile -t rby1-ros2:humble-arm64 .
#
# The rby1-sdk layer is built with the same script used for local (native) installs.
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base AS sdk

ARG RBY1_SDK_VERSION=v0.10.0
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git python3-venv python3-pip pkg-config \
    && rm -rf /var/lib/apt/lists/*
COPY scripts/install_rby1_sdk.sh /tmp/install_rby1_sdk.sh
RUN bash /tmp/install_rby1_sdk.sh --prefix /opt/rby1-sdk --version ${RBY1_SDK_VERSION} \
    && rm -rf /root/.conan2/p /root/.cache/rby1-sdk

FROM sdk AS workspace
ENV CMAKE_PREFIX_PATH=/opt/rby1-sdk \
    LD_LIBRARY_PATH=/opt/rby1-sdk/lib
WORKDIR /ws
# Dependencies first so source edits do not invalidate the rosdep layer.
COPY . /ws/src/rby1-ros2
RUN apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
