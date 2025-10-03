# ROS 2 development container for the Psyched workspace
# Base: Ubuntu 22.04 + ROS 2 Humble (ros-base)
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    DENO_TLS_CA_STORE=system \
    PSYCHED_AUTO_ACTIVATE=0

# Core tools expected by ./setup and psh installers
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      sudo bash curl ca-certificates gnupg lsb-release locales \
      build-essential make git unzip ripgrep \
      python3-full python3-pip python3-venv python-is-python3 \
      shellcheck micro mc && \
    rm -rf /var/lib/apt/lists/*

# Prepare locale used by provisioning and ROS scripts
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Default workspace location inside the container
WORKDIR /workspace/psyched

# Keep container alive by default for interactive use; commands are supplied via docker compose
CMD ["bash", "-lc", "exec bash"]

