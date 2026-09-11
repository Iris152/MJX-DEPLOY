#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export PYTHONPATH="$ROOT/python:${PYTHONPATH:-}"

IFACE="${1:-eth0}"
python3 -m mjx_deploy.ahac_go2_deploy stand-example \
  --interface "$IFACE" \
  --build-if-missing

