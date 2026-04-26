#!/usr/bin/env bash
# Quick robot-side diagnostics for OpenCR/LiDAR/GPS serial mapping and NMEA integrity.
# Usage:
#   scripts/debug/diagnose_robot_serial_gps.sh
#   scripts/debug/diagnose_robot_serial_gps.sh --seconds 8
#   scripts/debug/diagnose_robot_serial_gps.sh --gps-only

set -euo pipefail

SAMPLE_SECONDS=6
GPS_ONLY=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --seconds)
      SAMPLE_SECONDS="${2:-}"
      shift 2
      ;;
    --gps-only)
      GPS_ONLY=true
      shift
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if ! [[ "${SAMPLE_SECONDS}" =~ ^[0-9]+$ ]] || [[ "${SAMPLE_SECONDS}" -lt 1 ]]; then
  echo "Invalid --seconds value: ${SAMPLE_SECONDS} (expected integer >= 1)" >&2
  exit 2
fi

serial_ports=(/dev/opencr /dev/tb3_lidar /dev/gps1 /dev/gps2)
gps_ports=(/dev/gps1 /dev/gps2)

hr() { printf '%*s\n' 72 '' | tr ' ' '-'; }

print_port_mapping() {
  local port="$1"
  if [[ -e "${port}" ]]; then
    local real
    real="$(readlink -f "${port}" || true)"
    printf "%-12s -> %s\n" "${port}" "${real:-<unresolved>}"
  else
    printf "%-12s -> %s\n" "${port}" "<missing>"
  fi
}

print_id_properties() {
  local dev="$1"
  if [[ -e "${dev}" ]]; then
    echo "Properties for ${dev}:"
    udevadm info -q property -n "${dev}" | rg "ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_PATH|DEVNAME" || true
  fi
}

sample_gps() {
  local port="$1"
  local baud="$2"
  if [[ ! -e "${port}" ]]; then
    echo "[skip] ${port} missing"
    return 0
  fi
  echo "[sample] ${port} @ ${baud} for ${SAMPLE_SECONDS}s"
  # Capture short raw sample to detect fragmented/corrupt NMEA.
  timeout "${SAMPLE_SECONDS}"s stdbuf -o0 -e0 bash -lc "stty -F '${port}' ${baud} raw -echo -echoe -echok -echoctl -echoke -icrnl -onlcr && cat '${port}'" \
    | tr -d '\r' \
    | awk 'NR<=20 {print}'
}

echo "Robot serial/GPS diagnostic"
hr

echo "Symlink/device mapping:"
if [[ "${GPS_ONLY}" == true ]]; then
  for p in "${gps_ports[@]}"; do print_port_mapping "${p}"; done
else
  for p in "${serial_ports[@]}"; do print_port_mapping "${p}"; done
fi

hr
echo "udev identity hints:"
if [[ "${GPS_ONLY}" == true ]]; then
  for p in "${gps_ports[@]}"; do
    real="$(readlink -f "${p}" 2>/dev/null || true)"
    [[ -n "${real}" ]] && print_id_properties "${real}"
  done
else
  for p in "${serial_ports[@]}"; do
    real="$(readlink -f "${p}" 2>/dev/null || true)"
    [[ -n "${real}" ]] && print_id_properties "${real}"
  done
fi

hr
echo "Duplicate target check:"
declare -A target_to_aliases=()
for p in "${gps_ports[@]}"; do
  real="$(readlink -f "${p}" 2>/dev/null || true)"
  [[ -n "${real}" ]] || continue
  if [[ -n "${target_to_aliases[${real}]:-}" ]]; then
    target_to_aliases["${real}"]+=",${p}"
  else
    target_to_aliases["${real}"]="${p}"
  fi
done

dup_found=false
for real in "${!target_to_aliases[@]}"; do
  aliases="${target_to_aliases[${real}]}"
  if [[ "${aliases}" == *,* ]]; then
    dup_found=true
    echo "WARNING: ${aliases} all resolve to ${real}"
  fi
done
if [[ "${dup_found}" == false ]]; then
  echo "OK: /dev/gps1 and /dev/gps2 resolve to distinct devices (if both exist)."
fi

hr
echo "Raw NMEA sample (first 20 lines per port):"
sample_gps /dev/gps1 115200 || true
sample_gps /dev/gps2 115200 || true

hr
echo "If NMEA lines are fragmented/garbled:"
echo "1) test one GPS at a time (disable gps_enable_2 or gps_enable_1),"
echo "2) verify real baud per receiver (often 9600 or 115200),"
echo "3) check USB power/noise/hub/cables and CP2102 adapters."
