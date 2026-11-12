#!/usr/bin/env bash
set -eo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/humble/setup.bash
exec ros2 launch "$here/capture.launch.py" "$@"
