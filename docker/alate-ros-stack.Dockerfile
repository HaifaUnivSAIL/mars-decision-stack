FROM ros:humble AS builder

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ffmpeg \
    git \
    libboost-all-dev \
    libzmq3-dev \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-ament-cmake \
    ros-humble-geometry-msgs \
    ros-humble-rosidl-default-generators \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir pyzmq lxml

WORKDIR /workspace

COPY external/nemala_core /workspace/external/nemala_core
COPY external/alate /workspace/external/alate
COPY ws /workspace/ws

RUN cmake -S /workspace/external/nemala_core/src -B /tmp/nemala-core-build && \
    cmake --build /tmp/nemala-core-build --parallel && \
    cmake --install /tmp/nemala-core-build

RUN python3 -m pip install --no-cache-dir -r /workspace/external/alate/src/HighLevelControl/AutoPilots/requirements.txt && \
    cmake -S /workspace/external/alate/src -B /tmp/alate-build \
      -DNeMALA_INCLUDE_DIR=/usr/local/include \
      -DNeMALA_LIBRARY_PATH=/usr/local/lib \
      -DCMAKE_INSTALL_PREFIX=/opt/alate \
      -DALATE_BUILD_MC=ON \
      -DALATE_BUILD_HLC=ON \
      -DALATE_BUILD_OSC=ON \
      -DALATE_BUILD_BMA=ON && \
    cmake --build /tmp/alate-build --parallel && \
    cmake --install /tmp/alate-build

RUN source /opt/ros/humble/setup.bash && \
    cd /workspace/ws && \
    colcon build --merge-install

FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ffmpeg \
    libboost-all-dev \
    libzmq5 \
    python3-pip \
    python3-pytest \
    ros-humble-geometry-msgs \
    ros-humble-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /usr/local /usr/local
COPY --from=builder /opt/alate /opt/alate
COPY --from=builder /workspace/ws/install /opt/ros2_ws/install

RUN ldconfig

WORKDIR /workspace
CMD ["bash"]
