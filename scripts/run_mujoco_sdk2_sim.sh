#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export PYTHONPATH="$ROOT/python:${PYTHONPATH:-}"

# 启动本机 MuJoCo 仿真低层：发布 rt/lowstate，订阅 rt/lowcmd。
python3 -m mjx_deploy.ahac_go2_deploy sim \
  --interface lo \
  --domain-id 1 \
  --build-if-missing \
  "$@"
