#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$(realpath "$0")")"
exec .venv/bin/python robot.py --hardware --config robot.local.yaml "$@"
