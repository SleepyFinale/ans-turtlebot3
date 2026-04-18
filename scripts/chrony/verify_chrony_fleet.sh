#!/usr/bin/env bash
# Print Chrony sync status and clock offset (run on robot or central PC).
set -euo pipefail

echo "=== timedatectl ==="
timedatectl status || true

if command -v chronyc >/dev/null 2>&1; then
  echo ""
  echo "=== chronyc tracking ==="
  chronyc tracking || true
  echo ""
  echo "=== chronyc sources (-v) ==="
  chronyc sources -v || true
else
  echo "chronyc not found; install chrony (scripts/chrony/setup_chrony_fleet_client.sh)." >&2
  exit 1
fi
