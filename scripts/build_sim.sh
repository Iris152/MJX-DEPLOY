#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export PYTHONPATH="$ROOT/python:${PYTHONPATH:-}"

# 构建 C++ MuJoCo + SDK2 仿真低层，默认输出到 build-sim。
python3 -m mjx_deploy.ahac_go2_deploy build-sim "$@"
