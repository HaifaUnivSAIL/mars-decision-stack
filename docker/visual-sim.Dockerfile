FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV GZ_VERSION=harmonic
ENV QT_X11_NO_MITSHM=1
ARG ARDUPILOT_REF=bbd1d94d4efa8ed8db94c20bbabeadd7fd5c846b
SHELL ["/bin/bash", "-lc"]
USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    git \
    gnupg \
    gstreamer1.0-gl \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-bad \
    libdebuginfod1 \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libopencv-dev \
    lsb-release \
    pkg-config \
    python3-dev \
    python3-pexpect \
    python3-future \
    python3-lxml \
    python3-pip \
    python3-setuptools \
    python3-venv \
    python3-wheel \
    rapidjson-dev \
    rsync \
    socat \
    wget \
    xauth \
    && rm -rf /var/lib/apt/lists/*

RUN wget -qO /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg https://packages.osrfoundation.org/gazebo.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic \
      libgz-sim8-dev \
      && rm -rf /var/lib/apt/lists/*

COPY external/ardupilot_gazebo /opt/ardupilot_gazebo
COPY sim /opt/mars-sim
COPY docker/visual-sim-entrypoint.sh /usr/local/bin/visual-sim-entrypoint.sh

RUN git clone https://github.com/ArduPilot/ardupilot.git /opt/ardupilot && \
    cd /opt/ardupilot && \
    git checkout "${ARDUPILOT_REF}" && \
    git submodule update --init --recursive && \
    ./waf configure --board sitl && \
    ./waf copter --jobs "$(nproc)"

RUN cmake -S /opt/ardupilot_gazebo -B /opt/ardupilot_gazebo/build -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    cmake --build /opt/ardupilot_gazebo/build --parallel && \
    chmod +x /usr/local/bin/visual-sim-entrypoint.sh

ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
ENV GZ_SIM_RESOURCE_PATH=/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds:/opt/mars-sim/models:/opt/mars-sim/worlds:${GZ_SIM_RESOURCE_PATH}

ENTRYPOINT ["/usr/local/bin/visual-sim-entrypoint.sh"]
