#!/usr/bin/env bash
set -eo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
cd "$here/.."
colcon build --base-paths "$here/../GRIL-Calib" --build-base "$here/../gril_ws/build" \
  --install-base "$here/../gril_ws/install" --packages-select gril_calib \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DGRIL_WITH_LIVOX=OFF \
  -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
ctest --test-dir "$here/../gril_ws/build/gril_calib" --output-on-failure
