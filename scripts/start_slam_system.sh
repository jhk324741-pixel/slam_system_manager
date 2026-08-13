#!/usr/bin/env bash
set -eo pipefail

readonly slam_workspace="/home/nvidia/fast_lio_ouster_ws"
readonly ros_setup="/opt/ros/humble/setup.bash"
readonly workspace_setup="${slam_workspace}/install/setup.bash"
readonly deployment_config="${slam_workspace}/install/slam_system_manager/share/slam_system_manager/config/process_deployment.yaml"

if [[ ! -r "${ros_setup}" ]]; then
  echo "ROS 2 setup not found: ${ros_setup}" >&2
  exit 1
fi
if [[ ! -r "${workspace_setup}" ]]; then
  echo "Workspace setup not found: ${workspace_setup}" >&2
  exit 1
fi
if [[ ! -r "${deployment_config}" ]]; then
  echo "Deployment process config not found: ${deployment_config}" >&2
  exit 1
fi

source "${ros_setup}"
source "${workspace_setup}"
set -u
cd "${slam_workspace}"

exec ros2 launch slam_system_manager system_bringup.launch.py \
  process_config_file:="${deployment_config}"
