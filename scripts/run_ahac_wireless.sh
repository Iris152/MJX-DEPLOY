#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export PYTHONPATH="$ROOT/python:${PYTHONPATH:-}"

# 第一个参数是网卡名，第二个参数是 DDS 域编号。
IFACE="${1:-eth0}"
DOMAIN_ID="${2:-0}"
# 无线模式直接读取 Unitree 遥控器摇杆作为速度指令。
python3 -m mjx_deploy.ahac_go2_deploy run \
  --interface "$IFACE" \
  --domain-id "$DOMAIN_ID" \
  --command-source wireless \
  --build-if-missing
