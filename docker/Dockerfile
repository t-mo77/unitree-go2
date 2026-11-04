# Shared image for `app` (Web + brain) and `sim` (fake Go2): go2-voice-ltl:dev
# Ubuntu 22.04 + ROS 2 Humble, like the PC Unitree recommends for a real Go2.
# Builds on arm64 (Apple Silicon Mac) and x86_64 (Linux PC).
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Pinned versions (checked on 2026-11-04)
ARG NODE_VERSION=22.23.4-1nodesource1
ARG UNITREE_ROS2_COMMIT=668d1ec5a05d1c38d3306bdca7d59f2ba3581a88
ARG CLAUDE_CODE_VERSION=2.1.303
ARG CODEX_VERSION=0.166.0

# ROS 2 / system packages
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rosidl-generator-dds-idl \
        ros-humble-sensor-msgs-py \
        python3-pip \
        graphviz \
        git \
        curl \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Node.js 22 (NodeSource) for the Codex CLI
RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash - \
    && apt-get install -y --no-install-recommends "nodejs=${NODE_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

# unitree_ros2 messages (unitree_api, unitree_go). On Humble the stock
# rmw_cyclonedds_cpp is used, so cyclonedds itself is not built (official steps).
RUN git clone https://github.com/unitreerobotics/unitree_ros2 /root/unitree_ros2 \
    && cd /root/unitree_ros2 \
    && git checkout "${UNITREE_ROS2_COMMIT}" \
    && cd cyclonedds_ws \
    && source /opt/ros/humble/setup.bash \
    && colcon build --packages-select unitree_api unitree_go \
    && rm -rf build log

# Python packages (pytest stays on 8.x: pytest 9 rejects the hook signature of
# ROS Humble's launch_testing plugin, which every pytest run loads)
RUN pip3 install --no-cache-dir \
        numpy==1.26.4 \
        pitct==1.0.8 \
        flloat==0.3.0 \
        faster-whisper==1.2.1 \
        ctranslate2==4.8.5 \
        fastapi==0.162.0 \
        "uvicorn[standard]==0.54.0" \
        python-multipart==0.0.32 \
        httpx==0.28.1 \
        pytest==8.4.2

# Claude Code CLI (native installer, fixed version; no self-update in the image)
ENV PATH=/root/.local/bin:${PATH}
RUN curl -fsSL https://claude.ai/install.sh | bash -s "${CLAUDE_CODE_VERSION}"
ENV DISABLE_UPDATES=1

# Codex CLI
RUN npm install -g "@openai/codex@${CODEX_VERSION}" \
    && rm -rf /root/.npm

# Our setup.sh replaces the upstream one (which is written for foxy + enp3s0)
COPY unitree_setup.sh /root/unitree_ros2/setup.sh

ENV PYTHONPATH=/work:/work/nl_control
WORKDIR /work

ENTRYPOINT ["bash", "-lc", "source ~/unitree_ros2/setup.sh && exec \"$@\"", "--"]
CMD ["bash"]
