#!/usr/bin/env bash
set -euo pipefail

OPENCR_PORT="${1:-/dev/opencr}"

have_cmd() {
  command -v "$1" >/dev/null 2>&1
}

section() {
  echo
  echo "=== $1 ==="
}

echo "OpenCR diagnostic report"
echo "Target port: ${OPENCR_PORT}"
echo "Timestamp: $(date -Iseconds)"

section "Path and symlink status"
if [[ -e "${OPENCR_PORT}" ]]; then
  ls -l "${OPENCR_PORT}" || true
  REAL_PORT="$(readlink -f "${OPENCR_PORT}" || true)"
  echo "Resolved target: ${REAL_PORT:-<unresolved>}"
else
  echo "ERROR: ${OPENCR_PORT} does not exist."
  echo "Hint: verify udev rules and replug OpenCR USB."
  exit 2
fi

section "Character-device and permission checks"
if [[ -c "${OPENCR_PORT}" ]]; then
  echo "${OPENCR_PORT} is a character device."
else
  echo "WARNING: ${OPENCR_PORT} is not a character device."
fi
id || true
stat "${OPENCR_PORT}" || true

section "Device identity (udev)"
if have_cmd udevadm; then
  udevadm info -q property -n "${OPENCR_PORT}" | \
    awk '/^(DEVNAME|ID_VENDOR|ID_MODEL|ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL|ID_PATH|ID_MM_DEVICE_IGNORE)=/ {print}'
else
  echo "udevadm not found."
fi

section "TTY inventory"
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "No ttyACM/ttyUSB devices visible."

section "Rule files likely relevant"
ls -l /etc/udev/rules.d/*opencr* /etc/udev/rules.d/*turtlebot3* 2>/dev/null || \
  echo "No OpenCR/TurtleBot3-named udev rule files found."

section "ModemManager status"
if have_cmd systemctl; then
  systemctl is-active ModemManager 2>/dev/null || true
  systemctl is-enabled ModemManager 2>/dev/null || true
else
  echo "systemctl not available in this environment."
fi

section "Potential port users"
if have_cmd lsof; then
  lsof "${OPENCR_PORT}" || echo "No process currently holds ${OPENCR_PORT}."
elif have_cmd fuser; then
  fuser -v "${OPENCR_PORT}" || echo "No process currently holds ${OPENCR_PORT}."
else
  echo "Neither lsof nor fuser is installed."
fi

section "Quick recommendations"
echo "- Expected OpenCR USB IDs are typically 0483:5740 (runtime) or 0483:df11 (bootloader)."
echo "- Ensure udev sets ID_MM_DEVICE_IGNORE=1 for OpenCR to avoid ModemManager grabs."
echo "- If ${OPENCR_PORT} is missing/intermittent: reload udev rules, re-trigger, and replug USB."
echo
echo "Done."
