#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export PYTHONPATH="$ROOT/python:${PYTHONPATH:-}"

IFACE="${1:-eth0}"
DOMAIN_ID="${2:-0}"
python3 -m mjx_deploy.ahac_go2_deploy run \
  --interface "$IFACE" \
  --domain-id "$DOMAIN_ID" \
  --command-source terminal \
  --build-if-missing

