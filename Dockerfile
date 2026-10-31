FROM ubuntu:22.04 AS xrce-agent-builder

# ROS 2 Humble ships Fast DDS 2.6.x. PX4's compatibility matrix pairs it
# with Micro XRCE-DDS Agent v2.4.2. Keep both revisions reproducible.
ARG XRCE_AGENT_VERSION=v2.4.2
ARG FASTDDS_TAG=v2.12.2
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        git \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch "${XRCE_AGENT_VERSION}" \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /src/agent \
    && sed -i "s/set(_fastdds_tag 2.12.x)/set(_fastdds_tag ${FASTDDS_TAG})/" \
        /src/agent/CMakeLists.txt \
    && cmake -S /src/agent -B /src/agent/build \
        -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /src/agent/build --parallel 4 \
    && cmake --install /src/agent/build


FROM ubuntu:22.04 AS xrce-agent

RUN apt-get update && apt-get install -y --no-install-recommends \
        libstdc++6 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=xrce-agent-builder /usr/local/ /usr/local/
RUN ldconfig

ENTRYPOINT ["MicroXRCEAgent"]


FROM ros:humble-ros-base AS ros-runtime

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8

# Retry downloads on slow Raspberry Pi links. Ubuntu Ports uses HTTPS; the ROS
# mirror remains HTTP because its current certificate does not match the host.
RUN printf '%s\n' \
      'Acquire::Retries "10";' \
      'Acquire::http::Timeout "30";' \
      'Acquire::https::Timeout "30";' \
      'Acquire::http::Pipeline-Depth "0";' \
      > /etc/apt/apt.conf.d/80-retries \
    && find /etc/apt/sources.list /etc/apt/sources.list.d -type f \
      -exec sed -i \
      's|http://ports.ubuntu.com/ubuntu-ports|https://ports.ubuntu.com/ubuntu-ports|g' \
      {} +

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-opencv \
        python3-pip \
        ros-humble-camera-calibration \
        ros-humble-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

# ZipDepth CPU backend. Pin NumPy below 2 because Ubuntu's OpenCV bindings are
# built against NumPy 1.x.
RUN python3 -m pip install --no-cache-dir \
        numpy==1.26.4 \
        onnxruntime==1.19.2

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' \
      >> /root/.bashrc

WORKDIR /ros2_ws
