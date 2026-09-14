#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# 第一个参数可指定构建目录，默认使用 build。
BUILD_DIR="${1:-build}"
# 默认构建非 ROS2 版本，同时编译宇树起立示例。
cmake -S . -B "$BUILD_DIR" -DMJX_DEPLOY_ENABLE_ROS2=OFF -DMJX_DEPLOY_BUILD_STAND_EXAMPLE=ON
cmake --build "$BUILD_DIR" -j"$(nproc)"
