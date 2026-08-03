#!/bin/bash
# 保存当前 RTAB-Map 2D 占据网格图用于 Nav2
#
# 用法:
#   ./save_rtabmap_map.sh                         # 保存为 maps/rtabmap_nav2_latest.{yaml,pgm}
#   ./save_rtabmap_map.sh /grid_prob_map          # 指定占据栅格话题
#   ./save_rtabmap_map.sh rtabmap_nav2 /grid_prob_map # 兼容旧用法, 第一个参数会被忽略
#   ros2 run nav2_map_server map_saver_cli -t /map -f mymap   # 手动替代
#
# 前提条件: RTAB-Map SLAM / 定位模式必须在运行中
#   ros2 launch velodyne_bringup rtabmap_slam.launch.py
#
# RTAB-Map 在 Grid/FromDepth=false 等配置下通常发布 /map
# 有些配置会发布 /proj_map 或 /grid_prob_map
# (nav_msgs/OccupancyGrid), map_saver_cli 订阅此话题并保存为
# PGM+YAML 格式, 可直接被 Nav2 map_server 加载.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
MAP_DIR="${PROJECT_DIR}/maps"

# 若 maps/ 不存在则创建
mkdir -p "${MAP_DIR}"

OUTPUT_NAME="rtabmap_nav2_latest"
OUTPUT_PATH="${MAP_DIR}/${OUTPUT_NAME}"
TMP_NAME=".${OUTPUT_NAME}_tmp_$$"
TMP_PATH="${MAP_DIR}/${TMP_NAME}"

MAP_TOPIC="${2:-}"
if [ -z "${MAP_TOPIC}" ] && [[ "${1:-}" == /* ]]; then
    MAP_TOPIC="${1}"
fi

echo "========================================"
echo " RTAB-Map → Nav2 地图导出"
echo "========================================"
echo "输出目录: ${MAP_DIR}"
echo "文件名:   ${OUTPUT_NAME}.{yaml,pgm}"
echo ""

# 自动选择 RTAB-Map 发布的 2D 占据栅格话题。
# 使用 --no-daemon 避免 ROS daemon 缓存导致看到已经失效的话题。
if [ -n "${MAP_TOPIC}" ]; then
    CANDIDATE_TOPICS=("${MAP_TOPIC}")
else
    TOPICS="$(ros2 topic list --no-daemon --spin-time 2 2>/dev/null || true)"
    CANDIDATE_TOPICS=()
    for TOPIC in /map /proj_map /grid_prob_map; do
        if echo "${TOPICS}" | grep -qx "${TOPIC}"; then
            CANDIDATE_TOPICS+=("${TOPIC}")
        fi
    done
fi

# 检查地图话题是否存在
if [ "${#CANDIDATE_TOPICS[@]}" -eq 0 ]; then
    echo "[错误] 找不到可保存的 2D 占据栅格话题"
    echo ""
    echo "请确保 RTAB-Map SLAM / 定位模式正在运行:"
    echo "  ros2 launch velodyne_bringup rtabmap_slam.launch.py"
    echo "  (或者 rtabmap_localization.launch.py)"
    echo ""
    echo "也可以手动指定话题:"
    echo "  ${0} rtabmap_nav2 /map"
    echo "  ${0} rtabmap_nav2 /proj_map"
    echo "  ${0} rtabmap_nav2 /grid_prob_map"
    echo ""
    echo "运行中的话题列表:"
    ros2 topic list 2>/dev/null || echo "  (ROS2 未运行)"
    exit 1
fi

echo "[1/2] 保存 2D 占据网格 ..."
echo "      候选话题: ${CANDIDATE_TOPICS[*]}"
echo "      文件: ${OUTPUT_NAME}.{yaml,pgm}"

SAVED_TOPIC=""
for TOPIC in "${CANDIDATE_TOPICS[@]}"; do
    echo "      尝试话题: ${TOPIC}"
    if ros2 run nav2_map_server map_saver_cli \
        -t "${TOPIC}" \
        -f "${TMP_PATH}" \
        --mode trinary \
        --occ 0.65 \
        --free 0.25; then
        SAVED_TOPIC="${TOPIC}"
        break
    fi
    rm -f "${TMP_PATH}.yaml" "${TMP_PATH}.pgm"
done

if [ -z "${SAVED_TOPIC}" ]; then
    echo "[错误] 候选话题存在，但 map_saver 没有收到可保存的地图消息"
    echo "请等待 RTAB-Map 更新地图后重试，或手动指定当前正在发布的 OccupancyGrid 话题。"
    exit 1
fi

echo "      已保存话题: ${SAVED_TOPIC}"

echo "[2/2] 写入 rtabmap_nav2_latest 地图文件 ..."
sed -i "s|^image: .*|image: ${OUTPUT_NAME}.pgm|" "${TMP_PATH}.yaml"
mv -f "${TMP_PATH}.pgm" "${OUTPUT_PATH}.pgm"
mv -f "${TMP_PATH}.yaml" "${OUTPUT_PATH}.yaml"

# 同时备份数据库 (方便后续重新导出)
DB_SRC="${HOME}/.ros/rtabmap.db"
if [ -f "${DB_SRC}" ]; then
    cp "${DB_SRC}" "${OUTPUT_PATH}.db"
    echo "     数据库备份: ${OUTPUT_NAME}.db"
fi

echo ""
echo "========================================"
echo " 完成！"
echo "========================================"
echo "  地图文件:"
echo "    ${OUTPUT_PATH}.yaml"
echo "    ${OUTPUT_PATH}.pgm"
echo ""
echo "  启动定位+导航:"
echo "    ros2 launch velodyne_bringup rtabmap_nav_complete.launch.py"
echo "  或单独导航:"
echo "    ros2 launch velodyne_bringup nav2_navigation.launch.py"
echo ""
