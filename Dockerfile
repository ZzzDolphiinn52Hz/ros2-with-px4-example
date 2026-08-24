FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8

# Hotspot and some ISPs drop HTTP to packages.ros.org (timeout on :80).
# Retry and force HTTPS so apt can finish on a slow link.
RUN printf '%s\n' \
      'Acquire::Retries "10";' \
      'Acquire::http::Timeout "30";' \
      'Acquire::https::Timeout "30";' \
      'Acquire::http::Pipeline-Depth "0";' \
      > /etc/apt/apt.conf.d/80-retries \
 && find /etc/apt/sources.list /etc/apt/sources.list.d -type f \
      -exec sed -i 's|http://packages.ros.org/ros2/ubuntu|https://packages.ros.org/ros2/ubuntu|g' {} +

# Ubuntu ports first — these usually succeed even when the ROS repo is flaky.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        python3-colcon-common-extensions \
        python3-pip \
        python3-numpy \
        python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# ZipDepth CPU backend. Keep the inference runtime outside the ROS dependency
# graph so the same ONNX model can be profiled independently on ARM64.
RUN python3 -m pip install --no-cache-dir \
        numpy==1.26.4 \
        onnxruntime==1.19.2

# Separate layer: if this times out, rebuild only retries this step.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-slam-toolbox \
        ros-humble-tf2-ros \
        ros-humble-tf2-tools \
        ros-humble-tf2-geometry-msgs \
        ros-humble-robot-state-publisher \
        ros-humble-camera-calibration \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws
