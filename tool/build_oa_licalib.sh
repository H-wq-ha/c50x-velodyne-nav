#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root"
docker build --progress=plain -f tool/oa_licalib.Dockerfile -t oa-licalib:noetic tool
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$root:/project" -w /project oa-licalib:noetic bash -c '
set -eo pipefail
source /opt/ros/noetic/setup.bash
prefix=/project/tool/oa_licalib_deps
source_path=/project/tool/OA-LICalib
cmake -S "$source_path/thirdparty/ceres-solver" -B /project/tool/oa_build_ceres \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix" \
  -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF \
  -DBUILD_SHARED_LIBS=ON -DCUDA=OFF
cmake --build /project/tool/oa_build_ceres -j4
cmake --install /project/tool/oa_build_ceres
cmake -S "$source_path/thirdparty/Pangolin" -B /project/tool/oa_build_pangolin \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix" \
  -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_TESTS=OFF \
  -DCMAKE_DISABLE_FIND_PACKAGE_Python=ON -DBUILD_PANGOLIN_FFMPEG=OFF \
  -DBUILD_PANGOLIN_LIBDC1394=OFF -DBUILD_PANGOLIN_OPENNI=OFF \
  -DBUILD_PANGOLIN_OPENNI2=OFF
cmake --build /project/tool/oa_build_pangolin -j4
cmake --build /project/tool/oa_build_pangolin --target pango_python -j4
cmake --install /project/tool/oa_build_pangolin
cd /project/tool/oa_licalib_ws
catkin_make -j2 -l4 -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_WHITELIST_PACKAGES="ndt_omp;rslidar_msgs" \
  -DCMAKE_PREFIX_PATH="$prefix;/opt/ros/noetic"
catkin_make -j2 -l4 -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_WHITELIST_PACKAGES="" \
  -DCMAKE_PREFIX_PATH="/project/tool/oa_licalib_ws/devel;$prefix;/opt/ros/noetic" \
  -DCeres_DIR="$prefix/lib/cmake/Ceres" \
  -DPangolin_DIR="$prefix/lib/cmake/Pangolin"
'
