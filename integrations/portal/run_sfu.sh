#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
server="${LIVEKIT_SERVER_BIN:-livekit-server}"
config="${1:-.sfu/livekit.local.yaml}"
if [[ ! -f "$config" ]]; then
  echo "Missing $config; first run .venv/bin/python configure_sfu.py" >&2
  exit 1
fi
if ! command -v "$server" >/dev/null 2>&1; then
  echo 'Set LIVEKIT_SERVER_BIN to your installed official livekit-server binary.' >&2
  exit 1
fi
exec "$server" --config "$config"
