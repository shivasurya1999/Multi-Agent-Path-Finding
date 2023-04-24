#!/bin/bash
set -e

echo "==> Executing master image entrypoint ..."

echo "-> Setting up ROS"
source "/opt/ros/noetic/setup.sh"

echo "==> Container ready"
exec "$@"