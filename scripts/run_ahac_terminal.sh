#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export PYTHONPATH="$ROOT/python:${PYTHONPATH:-}"

# 第一个参数是网卡名，第二个参数是 DDS 域编号。
IFACE="${1:-eth0}"
DOMAIN_ID="${2:-0}"
# 终端模式下使用 Enter 起立/进入行走，用 w/s/a/d/q/e 调速度。
python3 -m mjx_deploy.ahac_go2_deploy run \
  --interface "$IFACE" \
  --domain-id "$DOMAIN_ID" \
  --command-source terminal \
  --build-if-missing
