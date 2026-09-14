#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export PYTHONPATH="$ROOT/python:${PYTHONPATH:-}"

# 第一个参数是连接 Go2 的网卡名。
IFACE="${1:-eth0}"
# 先运行官方起立示例，确认低层 DDS 通信正常。
python3 -m mjx_deploy.ahac_go2_deploy stand-example \
  --interface "$IFACE" \
  --build-if-missing
