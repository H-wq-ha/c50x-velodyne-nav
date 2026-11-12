#!/usr/bin/env bash
set -eo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
source /opt/ros/humble/setup.bash
if [[ ! -f "$here/../gril_ws/install/setup.bash" ]]; then
  echo '尚未构建，请先运行 tool/gril_calibration/build.sh' >&2
  exit 1
fi
source "$here/../gril_ws/install/setup.bash"
exec /usr/bin/python3 "$here/run.py" "$@"
