#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export PYTHONPATH="$ROOT/python:${PYTHONPATH:-}"

# 连接本机仿真低层，使用真实 AHAC C++ 部署控制器做状态机和策略推理。
python3 -m mjx_deploy.ahac_go2_deploy run \
  --interface lo \
  --domain-id 1 \
  --command-source terminal \
  --build-if-missing \
  "$@"
