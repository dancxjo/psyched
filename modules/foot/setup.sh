#!/usr/bin/env bash
set -euo pipefail

ALLOW_MISSING="${PSH_ALLOW_MISSING_DEPENDENCIES:-0}"

check_tool() {
    if ! command -v "$1" >/dev/null 2>&1; then
        if [ "$ALLOW_MISSING" = "1" ]; then
            echo "Warning: '$1' not found; continuing due to PSH_ALLOW_MISSING_DEPENDENCIES=1" >&2
            return 0
        fi
        echo "Error: '$1' is required. $2" >&2
        exit 1
    fi
}

check_tool git "Install Git by following https://git-scm.com/downloads"
check_tool rosdep "Install rosdep (https://docs.ros.org) before continuing."
check_tool colcon "Install colcon via https://colcon.readthedocs.io"

REPO_DIR="$(pwd)"
SOURCE_DIR="${REPO_DIR}/src"
CREATE_ROBOT_REPO="https://github.com/autonomylab/create_robot.git"
LIBCREATE_REPO="https://github.com/revyos-ros/libcreate.git"
LIBCREATE_BRANCH="fix-std-string"
DISTRO="${ROS_DISTRO:-kilted}"

mkdir -p "$SOURCE_DIR"

CREATE_ROBOT_DIR="${SOURCE_DIR}/create_robot"
if [ ! -d "${CREATE_ROBOT_DIR}/.git" ]; then
    git clone "$CREATE_ROBOT_REPO" "$CREATE_ROBOT_DIR"
fi

LIBCREATE_DIR="${SOURCE_DIR}/libcreate"
if [ ! -d "${LIBCREATE_DIR}/.git" ]; then
    git clone --branch "$LIBCREATE_BRANCH" --single-branch "$LIBCREATE_REPO" "$LIBCREATE_DIR"
fi

WORKSPACE_PATH="$REPO_DIR" ROS_DISTRO="$DISTRO" bash -lc '
set -euo pipefail
source <(PSH_ENV_MODE=print psh env)
rosdep install -i --from-path src --rosdistro "${ROS_DISTRO}" -y
colcon build --symlink-install
colcon build --symlink-install --install-base install
'

SERVICE_NAME="psyched-foot.service"
SYSTEMD_DIR="${PSH_SYSTEMD_DIR:-/etc/systemd/system}"
SERVICE_PATH="${SYSTEMD_DIR}/${SERVICE_NAME}"
SERVICE_USER="${PSH_SERVICE_USER:-${USER:-root}}"

if command -v systemctl >/dev/null 2>&1; then
    chmod +x "${REPO_DIR}/modules/foot/launch.sh"
    mkdir -p "$(dirname "${SERVICE_PATH}")"
    cat >"${SERVICE_PATH}" <<EOF_SERVICE
[Unit]
Description=Psyched foot module bringup
After=network.target

[Service]
Type=simple
User=${SERVICE_USER}
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -lc 'cd ${REPO_DIR} && source <(WORKSPACE_PATH=${REPO_DIR} ROS_DISTRO=${DISTRO} PSH_ENV_MODE=print psh env) && ./modules/foot/launch.sh'
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOF_SERVICE

    systemctl daemon-reload
    systemctl enable --now "${SERVICE_NAME}"
    echo "Systemd service installed at ${SERVICE_PATH} and enabled via systemd."
else
    echo "systemctl not available; skipping systemd service installation."
fi
