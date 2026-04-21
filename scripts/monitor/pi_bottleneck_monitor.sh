#!/usr/bin/env bash
set -euo pipefail

INTERVAL_SEC="1.0"
IFACE="wlan0"
OUT_CSV=""
LINK_MBPS_OVERRIDE=""

usage() {
  cat <<'EOF'
Usage: pi_bottleneck_monitor.sh [options]

Options:
  --iface <name>         Network interface to monitor (default: wlan0)
  --interval <seconds>   Sampling interval in seconds (default: 1.0)
  --output <path>        Output CSV path (default: scripts/monitor/logs/pi_bottleneck_YYYYmmdd_HHMMSS.csv)
  --link-mbps <value>    Override link speed in Mbps if ethtool is unavailable
  --help                 Show this help

Example:
  ./scripts/monitor/pi_bottleneck_monitor.sh --iface wlan0 --interval 1.0
EOF
}

sanitize_csv_field() {
  echo "${1:-}" | tr ',' ';' | tr '\n' ' ' | xargs
}

parse_args() {
  while [ $# -gt 0 ]; do
    case "$1" in
      --iface)
        IFACE="${2:-}"
        shift 2
        ;;
      --interval)
        INTERVAL_SEC="${2:-}"
        shift 2
        ;;
      --output)
        OUT_CSV="${2:-}"
        shift 2
        ;;
      --link-mbps)
        LINK_MBPS_OVERRIDE="${2:-}"
        shift 2
        ;;
      --help|-h)
        usage
        exit 0
        ;;
      *)
        echo "Unknown option: $1" >&2
        usage
        exit 1
        ;;
    esac
  done
}

get_epoch_ms() {
  date +%s%3N
}

get_cpu_totals() {
  awk '/^cpu / {print $2,$3,$4,$5,$6,$7,$8,$9,$10,$11}' /proc/stat
}

get_net_counters() {
  awk -v iface="$IFACE" '
    $0 ~ ("^[[:space:]]*" iface ":") {
      gsub(":", "", $1)
      print $2, $4, $5, $10, $12, $13
      exit
    }
  ' /proc/net/dev
}

get_link_mbps() {
  if [ -n "$LINK_MBPS_OVERRIDE" ]; then
    echo "$LINK_MBPS_OVERRIDE"
    return
  fi

  if command -v ethtool >/dev/null 2>&1; then
    local speed
    speed="$(ethtool "$IFACE" 2>/dev/null | awk -F': ' '/Speed:/ {gsub(/Mb\/s/,"",$2); print $2; exit}')"
    if [ -n "${speed:-}" ] && [ "$speed" != "Unknown!" ]; then
      echo "$speed"
      return
    fi
  fi

  echo "100"
}

get_temp_c() {
  if command -v vcgencmd >/dev/null 2>&1; then
    local out
    out="$(vcgencmd measure_temp 2>/dev/null || true)"
    if [ -n "$out" ]; then
      echo "$out" | sed -n "s/temp=\([0-9.]*\)'C/\1/p"
      return
    fi
  fi

  if [ -r /sys/class/thermal/thermal_zone0/temp ]; then
    awk '{printf "%.2f", $1/1000.0}' /sys/class/thermal/thermal_zone0/temp
    return
  fi

  echo "0"
}

get_throttled_hex() {
  if command -v vcgencmd >/dev/null 2>&1; then
    vcgencmd get_throttled 2>/dev/null | sed -n 's/^throttled=0x//p'
  else
    echo "0"
  fi
}

bit_set() {
  local hex="$1"
  local bit="$2"
  local dec=$((16#$hex))
  if [ $(((dec >> bit) & 1)) -eq 1 ]; then
    echo "1"
  else
    echo "0"
  fi
}

get_mem_swap_stats() {
  awk '
    /^MemTotal:/ {mt=$2}
    /^MemAvailable:/ {ma=$2}
    /^SwapTotal:/ {st=$2}
    /^SwapFree:/ {sf=$2}
    END {
      mem_used=(mt-ma)
      mem_used_pct=(mt>0)?(100.0*mem_used/mt):0.0
      mem_avail_mb=ma/1024.0
      swap_used=(st-sf)
      swap_used_pct=(st>0)?(100.0*swap_used/st):0.0
      printf "%.2f %.2f %.2f\n", mem_used_pct, mem_avail_mb, swap_used_pct
    }
  ' /proc/meminfo
}

get_loadavg() {
  awk '{print $1,$2,$3}' /proc/loadavg
}

get_disk_used_pct() {
  df -P / | awk 'NR==2 {gsub("%","",$5); print $5}'
}

get_ssid() {
  if command -v iwgetid >/dev/null 2>&1; then
    iwgetid -r 2>/dev/null || true
  else
    echo ""
  fi
}

get_ip4() {
  ip -4 -o addr show dev "$IFACE" 2>/dev/null | awk '{print $4}' | head -1
}

parse_args "$@"

if [ -z "$OUT_CSV" ]; then
  mkdir -p "scripts/monitor/logs"
  OUT_CSV="scripts/monitor/logs/pi_bottleneck_$(date +%Y%m%d_%H%M%S).csv"
fi

mkdir -p "$(dirname "$OUT_CSV")"
LINK_MBPS="$(get_link_mbps)"

if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "Interface '$IFACE' not found." >&2
  exit 1
fi

echo "Logging bottleneck metrics to: $(basename "$OUT_CSV")"
echo "Interface: $IFACE  Interval: ${INTERVAL_SEC}s  Link Mbps: $LINK_MBPS"
echo "Stop with Ctrl+C."

cat > "$OUT_CSV" <<'EOF'
timestamp_iso,epoch_ms,iface,connected,ssid,ip4,cpu_usage_pct,load1,load5,load15,cpu_temp_c,throttled_hex,undervoltage_now,undervoltage_past,freq_capped_now,freq_capped_past,throttled_now,throttled_past,soft_temp_now,soft_temp_past,mem_used_pct,mem_avail_mb,swap_used_pct,disk_used_pct,rx_Bps,tx_Bps,rx_Mbps,tx_Mbps,link_mbps,rx_util_pct,tx_util_pct,rx_err_ps,tx_err_ps,rx_drop_ps,tx_drop_ps
EOF

RUNNING=1
trap 'RUNNING=0' INT TERM

read -r PREV_USER PREV_NICE PREV_SYSTEM PREV_IDLE PREV_IOWAIT PREV_IRQ PREV_SOFTIRQ PREV_STEAL PREV_GUEST PREV_GUEST_NICE <<<"$(get_cpu_totals)"
PREV_TOTAL=$((PREV_USER + PREV_NICE + PREV_SYSTEM + PREV_IDLE + PREV_IOWAIT + PREV_IRQ + PREV_SOFTIRQ + PREV_STEAL + PREV_GUEST + PREV_GUEST_NICE))
PREV_IDLE_ALL=$((PREV_IDLE + PREV_IOWAIT))

read -r PREV_RX_BYTES PREV_RX_ERR PREV_RX_DROP PREV_TX_BYTES PREV_TX_ERR PREV_TX_DROP <<<"$(get_net_counters)"
PREV_TS_MS="$(get_epoch_ms)"

while [ "$RUNNING" -eq 1 ]; do
  NOW_TS_MS="$(get_epoch_ms)"
  NOW_TS_ISO="$(date -Iseconds)"

  read -r USER NICE SYSTEM IDLE IOWAIT IRQ SOFTIRQ STEAL GUEST GUEST_NICE <<<"$(get_cpu_totals)"
  TOTAL=$((USER + NICE + SYSTEM + IDLE + IOWAIT + IRQ + SOFTIRQ + STEAL + GUEST + GUEST_NICE))
  IDLE_ALL=$((IDLE + IOWAIT))
  TOTALD=$((TOTAL - PREV_TOTAL))
  IDLED=$((IDLE_ALL - PREV_IDLE_ALL))
  if [ "$TOTALD" -gt 0 ]; then
    CPU_USAGE_PCT="$(awk -v t="$TOTALD" -v i="$IDLED" 'BEGIN {printf "%.2f", (100.0*(t-i))/t}')"
  else
    CPU_USAGE_PCT="0.00"
  fi

  PREV_TOTAL="$TOTAL"
  PREV_IDLE_ALL="$IDLE_ALL"

  read -r RX_BYTES RX_ERR RX_DROP TX_BYTES TX_ERR TX_DROP <<<"$(get_net_counters)"
  DT_MS=$((NOW_TS_MS - PREV_TS_MS))
  if [ "$DT_MS" -le 0 ]; then
    DT_MS=1
  fi

  RX_BPS="$(awk -v c="$RX_BYTES" -v p="$PREV_RX_BYTES" -v d="$DT_MS" 'BEGIN {v=(c-p)/(d/1000.0); if (v<0) v=0; printf "%.2f", v}')"
  TX_BPS="$(awk -v c="$TX_BYTES" -v p="$PREV_TX_BYTES" -v d="$DT_MS" 'BEGIN {v=(c-p)/(d/1000.0); if (v<0) v=0; printf "%.2f", v}')"
  RX_MBPS="$(awk -v b="$RX_BPS" 'BEGIN {printf "%.3f", (b*8.0)/1000000.0}')"
  TX_MBPS="$(awk -v b="$TX_BPS" 'BEGIN {printf "%.3f", (b*8.0)/1000000.0}')"

  RX_UTIL_PCT="$(awk -v m="$RX_MBPS" -v l="$LINK_MBPS" 'BEGIN {if (l<=0) {print "0.00"} else {printf "%.2f", 100.0*m/l}}')"
  TX_UTIL_PCT="$(awk -v m="$TX_MBPS" -v l="$LINK_MBPS" 'BEGIN {if (l<=0) {print "0.00"} else {printf "%.2f", 100.0*m/l}}')"

  RX_ERR_PS="$(awk -v c="$RX_ERR" -v p="$PREV_RX_ERR" -v d="$DT_MS" 'BEGIN {v=(c-p)/(d/1000.0); if (v<0) v=0; printf "%.3f", v}')"
  TX_ERR_PS="$(awk -v c="$TX_ERR" -v p="$PREV_TX_ERR" -v d="$DT_MS" 'BEGIN {v=(c-p)/(d/1000.0); if (v<0) v=0; printf "%.3f", v}')"
  RX_DROP_PS="$(awk -v c="$RX_DROP" -v p="$PREV_RX_DROP" -v d="$DT_MS" 'BEGIN {v=(c-p)/(d/1000.0); if (v<0) v=0; printf "%.3f", v}')"
  TX_DROP_PS="$(awk -v c="$TX_DROP" -v p="$PREV_TX_DROP" -v d="$DT_MS" 'BEGIN {v=(c-p)/(d/1000.0); if (v<0) v=0; printf "%.3f", v}')"

  PREV_RX_BYTES="$RX_BYTES"
  PREV_TX_BYTES="$TX_BYTES"
  PREV_RX_ERR="$RX_ERR"
  PREV_TX_ERR="$TX_ERR"
  PREV_RX_DROP="$RX_DROP"
  PREV_TX_DROP="$TX_DROP"
  PREV_TS_MS="$NOW_TS_MS"

  read -r LOAD1 LOAD5 LOAD15 <<<"$(get_loadavg)"
  read -r MEM_USED_PCT MEM_AVAIL_MB SWAP_USED_PCT <<<"$(get_mem_swap_stats)"
  DISK_USED_PCT="$(get_disk_used_pct)"
  CPU_TEMP_C="$(get_temp_c)"

  THROTTLED_HEX_RAW="$(get_throttled_hex)"
  THROTTLED_HEX="${THROTTLED_HEX_RAW:-0}"
  UV_NOW="$(bit_set "$THROTTLED_HEX" 0)"
  FREQ_CAP_NOW="$(bit_set "$THROTTLED_HEX" 1)"
  THROTTLED_NOW="$(bit_set "$THROTTLED_HEX" 2)"
  SOFT_TEMP_NOW="$(bit_set "$THROTTLED_HEX" 3)"
  UV_PAST="$(bit_set "$THROTTLED_HEX" 16)"
  FREQ_CAP_PAST="$(bit_set "$THROTTLED_HEX" 17)"
  THROTTLED_PAST="$(bit_set "$THROTTLED_HEX" 18)"
  SOFT_TEMP_PAST="$(bit_set "$THROTTLED_HEX" 19)"

  SSID="$(sanitize_csv_field "$(get_ssid)")"
  IP4="$(sanitize_csv_field "$(get_ip4)")"
  CONNECTED="0"
  if [ -n "$SSID" ]; then
    CONNECTED="1"
  fi

  printf "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,0x%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" \
    "$NOW_TS_ISO" "$NOW_TS_MS" "$IFACE" "$CONNECTED" "$SSID" "$IP4" \
    "$CPU_USAGE_PCT" "$LOAD1" "$LOAD5" "$LOAD15" "$CPU_TEMP_C" "$THROTTLED_HEX" \
    "$UV_NOW" "$UV_PAST" "$FREQ_CAP_NOW" "$FREQ_CAP_PAST" "$THROTTLED_NOW" "$THROTTLED_PAST" "$SOFT_TEMP_NOW" "$SOFT_TEMP_PAST" \
    "$MEM_USED_PCT" "$MEM_AVAIL_MB" "$SWAP_USED_PCT" "$DISK_USED_PCT" \
    "$RX_BPS" "$TX_BPS" "$RX_MBPS" "$TX_MBPS" "$LINK_MBPS" "$RX_UTIL_PCT" "$TX_UTIL_PCT" \
    "$RX_ERR_PS" "$TX_ERR_PS" "$RX_DROP_PS" "$TX_DROP_PS" \
    >> "$OUT_CSV"

  sleep "$INTERVAL_SEC"
done

CSV_BASENAME="$(basename "$OUT_CSV")"
ROS_LOG_BASENAME=""
if [[ "$CSV_BASENAME" == pi_bottleneck_*.csv ]]; then
  _stamp="${CSV_BASENAME#pi_bottleneck_}"
  _stamp="${_stamp%.csv}"
  ROS_LOG_BASENAME="ros_launch_${_stamp}.log"
fi

echo "Monitor stopped. Log written to: $CSV_BASENAME"
echo "Analyze with:"
if [[ -n "$ROS_LOG_BASENAME" ]]; then
  echo "  ./scripts/monitor/analyze_bottleneck_log.py $CSV_BASENAME --ros-log $ROS_LOG_BASENAME"
else
  echo "  ./scripts/monitor/analyze_bottleneck_log.py $CSV_BASENAME"
fi
