#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

BUILD_DIR="${1:-build}"
cmake -S . -B "$BUILD_DIR" -DMJX_DEPLOY_ENABLE_ROS2=OFF -DMJX_DEPLOY_BUILD_STAND_EXAMPLE=ON
cmake --build "$BUILD_DIR" -j"$(nproc)"

