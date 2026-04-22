#!/bin/bash
#
# Switch Raspberry Pi WiFi between SNS (lab), TAMU_WiFi (tamu), GCRI_LAB (gcri), ANS_starlink (star), RaspAP (rpi), and Azure (azure).
#
# Usage:
#   sudo ./scripts/wifi/switch_wifi.sh lab       # SNS WiFi with static IP (per robot/user)
#   sudo ./scripts/wifi/switch_wifi.sh tamu      # TAMU_WiFi (DHCP; PEAP + wpa patch per TAMU Pi KB)
#   sudo ./scripts/wifi/switch_wifi.sh gcri      # GCRI_LAB WiFi with static IP (per robot/user)
#   sudo ./scripts/wifi/switch_wifi.sh star      # ANS_starlink WiFi with static IP (per robot/user)
#   sudo ./scripts/wifi/switch_wifi.sh rpi       # RaspAP WiFi with static IP (per robot/user)
#   sudo ./scripts/wifi/switch_wifi.sh azure     # Azure hotspot with static IP (per robot/user)
#   ./scripts/wifi/switch_wifi.sh status         # show current WiFi (no sudo)
#
# Prereq: Remove or comment out the wifis/wlan0 block from
#   /etc/netplan/50-cloud-init.yaml so this script's 99-wifi-switch.yaml
#   is the only WiFi config (avoids "Duplicate access point SSID").
#
# Static IPs are chosen by current user (blinky / pinky / inky / clyde):
#   lab:    blinky@192.168.0.158,  pinky@192.168.0.194,   inky@192.168.0.139,   clyde@192.168.0.236
#   gcri:   blinky@192.168.50.158, pinky@192.168.50.194,  inky@192.168.50.139,  clyde@192.168.50.236
#   star:   blinky@192.168.1.158,  pinky@192.168.1.194,   inky@192.168.1.139,   clyde@192.168.1.236
#   rpi:    blinky@10.3.141.158,   pinky@10.3.141.194,    inky@10.3.141.139,    clyde@10.3.141.236
#   azure:  blinky@172.20.10.13,   pinky@172.20.10.14,    inky@172.20.10.15,    clyde@172.20.10.16
#   tamu:   TAMU_WiFi (DHCP; PEAP + wpa patch per TAMU Pi KB)
#
#   Optional: TAMU_CA_CERT=… (PEM) to validate RADIUS cert; TAMU_PASSWORD_NT_HASH=… for password=hash:… (KB 528)
#             TAMU_DOMAIN_SUFFIX=tamu.edu only if Help Desk says your cert needs domain_suffix_match
#   Debug:  Save WPA logs next to this script - use _WIFI_SCRIPT_DIR printed by "tamu" errors, or:
#           sudo journalctl -u netplan-wpa-wlan0.service -b --no-pager | tail -n 200 > scripts/wifi/netplan-wpa-wlan0.log
# When run with sudo we use SUDO_USER so "blinky" user gets the blinky IPs.
# Override with: $0 lab blinky / $0 gcri pinky / $0 star inky / $0 rpi clyde / $0 azure blinky or ROBOT_NAME=….
#

set -e
NETPLAN_OVERRIDE="/etc/netplan/99-wifi-switch.yaml"
_WIFI_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# SNS WiFi (lab) static IP config
LAB_SSID="SNS"
LAB_PASSWORD="sn5_rox!"
LAB_GATEWAY="192.168.0.1"
LAB_PREFIX="24"
LAB_IP_BLINKY="192.168.0.158"
LAB_IP_PINKY="192.168.0.194"
LAB_IP_INKY="192.168.0.139"
LAB_IP_CLYDE="192.168.0.236"

# GCRI_LAB WiFi (gcri) static IP config
GCRI_SSID="GCRI_LAB"
GCRI_PASSWORD="j1-gcri!"
GCRI_GATEWAY="192.168.50.1"
GCRI_PREFIX="24"
GCRI_IP_BLINKY="192.168.50.158"
GCRI_IP_PINKY="192.168.50.194"
GCRI_IP_INKY="192.168.50.139"
GCRI_IP_CLYDE="192.168.50.236"

# ANS_starlink WiFi (star) static IP config
STAR_SSID="ANS_starlink"
STAR_PASSWORD="an5_rox!"
STAR_GATEWAY="192.168.1.1"
STAR_PREFIX="24"
STAR_IP_BLINKY="192.168.1.158"
STAR_IP_PINKY="192.168.1.194"
STAR_IP_INKY="192.168.1.139"
STAR_IP_CLYDE="192.168.1.236"

# RaspAP WiFi (rpi) static IP config
RPI_SSID="RaspAP"
RPI_PASSWORD="sn5_rox!"
RPI_GATEWAY="10.3.141.1"
RPI_PREFIX="24"
RPI_IP_BLINKY="10.3.141.158"
RPI_IP_PINKY="10.3.141.194"
RPI_IP_INKY="10.3.141.139"
RPI_IP_CLYDE="10.3.141.236"

# Azure WiFi (azure) static IP config
AZURE_SSID="Azure"
AZURE_PASSWORD="howdoyouwanttodothis"
AZURE_GATEWAY="172.20.10.1"
AZURE_PREFIX="28"
AZURE_IP_BLINKY="172.20.10.13"
AZURE_IP_PINKY="172.20.10.14"
AZURE_IP_INKY="172.20.10.15"
AZURE_IP_CLYDE="172.20.10.16"

# TAMU campus WiFi (WPA Enterprise). SSID must match the network name (see TAMU WiFi KB).
TAMU_SSID="TAMU_WiFi"
TAMU_IDENTITY="schen08"
TAMU_PASSWORD="H0wdoyouwanttodothi$"
# Optional PEM for ca-certificate (unset = none, matching TAMU Pi KB 528). Example: USERTrust under /etc/ssl/certs.
TAMU_CA_CERT="${TAMU_CA_CERT:-}"

# Resolve robot name: current user (when using sudo we use SUDO_USER), or ROBOT_NAME env or second argument
get_robot_name() {
  local name
  name="${SUDO_USER:-$USER}"
  [ -n "$name" ] && name=$(echo "$name" | tr '[:upper:]' '[:lower:]')
  echo "${name:-}"
}

# Set per-robot static IPs for SNS, GCRI_LAB, ANS_starlink, RaspAP, and Azure. Exits if unknown robot.
set_robot_static_ips() {
  local robot
  robot="${1:-$(get_robot_name)}"
  robot="${robot,,}"
  case "$robot" in
    blinky)
      LAB_STATIC_IP="$LAB_IP_BLINKY"
      GCRI_STATIC_IP="$GCRI_IP_BLINKY"
      STAR_STATIC_IP="$STAR_IP_BLINKY"
      RPI_STATIC_IP="$RPI_IP_BLINKY"
      AZURE_STATIC_IP="$AZURE_IP_BLINKY"
      ;;
    pinky)
      LAB_STATIC_IP="$LAB_IP_PINKY"
      GCRI_STATIC_IP="$GCRI_IP_PINKY"
      STAR_STATIC_IP="$STAR_IP_PINKY"
      RPI_STATIC_IP="$RPI_IP_PINKY"
      AZURE_STATIC_IP="$AZURE_IP_PINKY"
      ;;
    inky)
      LAB_STATIC_IP="$LAB_IP_INKY"
      GCRI_STATIC_IP="$GCRI_IP_INKY"
      STAR_STATIC_IP="$STAR_IP_INKY"
      RPI_STATIC_IP="$RPI_IP_INKY"
      AZURE_STATIC_IP="$AZURE_IP_INKY"
      ;;
    clyde)
      LAB_STATIC_IP="$LAB_IP_CLYDE"
      GCRI_STATIC_IP="$GCRI_IP_CLYDE"
      STAR_STATIC_IP="$STAR_IP_CLYDE"
      RPI_STATIC_IP="$RPI_IP_CLYDE"
      AZURE_STATIC_IP="$AZURE_IP_CLYDE"
      ;;
    *)
      echo "Unknown robot: '$robot'. Current user is: $(get_robot_name)"
      echo "Use: $0 {lab|gcri|star|rpi|azure} {blinky|pinky|inky|clyde}   (or set ROBOT_NAME=…)"
      exit 1
      ;;
  esac
}

usage() {
  echo "Usage: $0 { lab | gcri | star | rpi | azure | tamu | status } [robot]"
  echo "  lab [blinky|pinky|inky|clyde]   - connect to SNS (static IP by robot)"
  echo "  gcri [blinky|pinky|inky|clyde]  - connect to GCRI_LAB (static IP by robot)"
  echo "  star [blinky|pinky|inky|clyde]  - connect to ANS_starlink (static IP by robot)"
  echo "  rpi [blinky|pinky|inky|clyde]   - connect to RaspAP (static IP by robot)"
  echo "  azure [blinky|pinky|inky|clyde] - connect to Azure (static IP by robot)"
  echo "  tamu                            - connect to TAMU_WiFi (DHCP; PEAP + wpa patch per TAMU Pi KB)"
  echo "  status                          - show current WiFi (no sudo)"
  exit 1
}

# Run netplan apply but hide the harmless Open vSwitch warning
netplan_apply_quiet() {
  netplan apply 2> >(grep -v -E 'Open vSwitch|ovsdb-server' >&2)
}

# Netplan cannot express TAMU-required wpa_supplicant fields (see TAMU KB article ID528). Patch generated conf and reload.
patch_wpa_supplicant_for_tamu() {
  local wpa tries u
  wpa="/run/netplan/wpa-wlan0.conf"
  tries=0
  while [ ! -f "$wpa" ] && [ "$tries" -lt 25 ]; do
    sleep 0.2
    tries=$((tries + 1))
  done
  if [ ! -f "$wpa" ]; then
    echo "Warning: $wpa missing after netplan apply; TAMU wpa tweaks not applied." >&2
    return 1
  fi
  if ! grep -q 'eap=PEAP' "$wpa"; then
    echo "Warning: $wpa has no PEAP block; skipping TAMU patch." >&2
    return 1
  fi
  TAMU_DOMAIN_SUFFIX="${TAMU_DOMAIN_SUFFIX:-}" \
  TAMU_PASSWORD_NT_HASH="${TAMU_PASSWORD_NT_HASH:-}" \
  python3 - "$wpa" <<'PY'
import os, re, sys
path = sys.argv[1]
domain = (os.environ.get("TAMU_DOMAIN_SUFFIX") or "").strip()
nt_hash = (os.environ.get("TAMU_PASSWORD_NT_HASH") or "").strip()

with open(path) as f:
    t = f.read()

out = t
# Script used to inject eapol_timeout= here; stock wpa on some images rejects it inside network={} (parse failure).
out = re.sub(r"\r?\n  eapol_timeout=\d+", "", out)

if 'phase1="peaplabel=0"' not in out:
    insert = '\n  proto=RSN\n  auth_alg=OPEN\n  phase1="peaplabel=0"\n  eap_workaround=1'
    if domain:
        insert += f"\n  domain_suffix_match={domain}"
    out, n = re.subn(r"(eap=PEAP)\r?\n", r"\1" + insert + "\n", out, count=1)
    if n != 1:
        print("patch_wpa: could not find eap=PEAP in", path, file=sys.stderr)
        sys.exit(1)
elif "eap_workaround=1" not in out:
    out, n = re.subn(
        r'(phase1="peaplabel=0")\r?\n',
        r"\1\n  eap_workaround=1\n",
        out,
        count=1,
    )
    if n != 1:
        print("patch_wpa: could not add eap_workaround=1 after phase1", file=sys.stderr)
        sys.exit(1)

# Netplan emits phase2="auth=mschapv2"; this build logs "TLS: Unsupported Phase2 EAP method 'mschapv2'" until MSCHAPV2.
out, _n = re.subn(
    r'phase2="auth=mschapv2"',
    'phase2="auth=MSCHAPV2"',
    out,
    flags=re.IGNORECASE,
)

if nt_hash:
    hexv = nt_hash
    if hexv.lower().startswith("(stdin)="):
        hexv = hexv.split("=", 1)[1].strip()
    hexv = hexv.strip()
    out, n = re.subn(r'password="[^"]*"', f"password=hash:{hexv}", out, count=1)
    if n != 1:
        print("patch_wpa: could not replace password= for NT hash", file=sys.stderr)
        sys.exit(1)

with open(path, "w") as f:
    f.write(out)
PY
}

restart_netplan_wpa() {
  local u
  u=$(systemctl list-units --type=service --no-legend 'netplan-wpa-*.service' 2>/dev/null | awk '{print $1}' | head -1)
  if [ -n "$u" ]; then
    systemctl restart "$u"
  else
    echo "Warning: no netplan-wpa-*.service; WiFi may need a reboot." >&2
  fi
}

write_netplan_lab() {
  cat << EOF
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: false
      addresses:
        - ${LAB_STATIC_IP}/${LAB_PREFIX}
      routes:
        - to: default
          via: ${LAB_GATEWAY}
      nameservers:
        addresses:
          - ${LAB_GATEWAY}
          - 8.8.8.8
      access-points:
        "${LAB_SSID}":
          password: "${LAB_PASSWORD}"
EOF
}

write_netplan_gcri() {
  cat << EOF
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: false
      addresses:
        - ${GCRI_STATIC_IP}/${GCRI_PREFIX}
      routes:
        - to: default
          via: ${GCRI_GATEWAY}
      nameservers:
        addresses:
          - ${GCRI_GATEWAY}
          - 8.8.8.8
      access-points:
        "${GCRI_SSID}":
          password: "${GCRI_PASSWORD}"
EOF
}

write_netplan_star() {
  cat << EOF
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: false
      addresses:
        - ${STAR_STATIC_IP}/${STAR_PREFIX}
      routes:
        - to: default
          via: ${STAR_GATEWAY}
      nameservers:
        addresses:
          - ${STAR_GATEWAY}
          - 8.8.8.8
      access-points:
        "${STAR_SSID}":
          password: "${STAR_PASSWORD}"
EOF
}

write_netplan_rpi() {
  cat << EOF
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: false
      addresses:
        - ${RPI_STATIC_IP}/${RPI_PREFIX}
      routes:
        - to: default
          via: ${RPI_GATEWAY}
      nameservers:
        addresses:
          - ${RPI_GATEWAY}
          - 8.8.8.8
      access-points:
        "${RPI_SSID}":
          password: "${RPI_PASSWORD}"
EOF
}

write_netplan_azure() {
  cat << EOF
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: false
      addresses:
        - ${AZURE_STATIC_IP}/${AZURE_PREFIX}
      routes:
        - to: default
          via: ${AZURE_GATEWAY}
      nameservers:
        addresses:
          - ${AZURE_GATEWAY}
          - 8.8.8.8
      access-points:
        "${AZURE_SSID}":
          password: "${AZURE_PASSWORD}"
EOF
}

write_netplan_tamu() {
  local ca_line=""
  if [ -n "${TAMU_CA_CERT:-}" ] && [ -r "${TAMU_CA_CERT}" ]; then
    ca_line="            ca-certificate: \"${TAMU_CA_CERT}\""
  fi
  cat << EOF
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "${TAMU_SSID}":
          auth:
            key-management: eap
            method: peap
            identity: "${TAMU_IDENTITY}"
            password: "${TAMU_PASSWORD}"
            phase2-auth: MSCHAPV2
${ca_line}
EOF
}

case "${1:-}" in
  lab)
    if [ "$(id -u)" -ne 0 ]; then
      echo "Run with sudo for SNS: sudo $0 lab"
      exit 1
    fi
    set_robot_static_ips "${2:-$ROBOT_NAME}"
    write_netplan_lab > "$NETPLAN_OVERRIDE"
    chmod 600 "$NETPLAN_OVERRIDE"
    netplan_apply_quiet
    echo "Switched to SNS (SSID ${LAB_SSID}, static IP ${LAB_STATIC_IP})."
    ;;
  gcri)
    if [ "$(id -u)" -ne 0 ]; then
      echo "Run with sudo for GCRI: sudo $0 gcri"
      exit 1
    fi
    set_robot_static_ips "${2:-$ROBOT_NAME}"
    write_netplan_gcri > "$NETPLAN_OVERRIDE"
    chmod 600 "$NETPLAN_OVERRIDE"
    netplan_apply_quiet
    echo "Switched to GCRI (SSID ${GCRI_SSID}, static IP ${GCRI_STATIC_IP})."
    ;;
  star)
    if [ "$(id -u)" -ne 0 ]; then
      echo "Run with sudo for Starlink: sudo $0 star"
      exit 1
    fi
    set_robot_static_ips "${2:-$ROBOT_NAME}"
    write_netplan_star > "$NETPLAN_OVERRIDE"
    chmod 600 "$NETPLAN_OVERRIDE"
    netplan_apply_quiet
    echo "Switched to Starlink (SSID ${STAR_SSID}, static IP ${STAR_STATIC_IP})."
    ;;
  rpi)
    if [ "$(id -u)" -ne 0 ]; then
      echo "Run with sudo for RaspAP: sudo $0 rpi"
      exit 1
    fi
    set_robot_static_ips "${2:-$ROBOT_NAME}"
    write_netplan_rpi > "$NETPLAN_OVERRIDE"
    chmod 600 "$NETPLAN_OVERRIDE"
    netplan_apply_quiet
    echo "Switched to RaspAP (SSID ${RPI_SSID}, static IP ${RPI_STATIC_IP})."
    ;;
  azure)
    if [ "$(id -u)" -ne 0 ]; then
      echo "Run with sudo for Azure: sudo $0 azure"
      exit 1
    fi
    set_robot_static_ips "${2:-$ROBOT_NAME}"
    write_netplan_azure > "$NETPLAN_OVERRIDE"
    chmod 600 "$NETPLAN_OVERRIDE"
    netplan_apply_quiet
    echo "Switched to Azure (SSID ${AZURE_SSID}, static IP ${AZURE_STATIC_IP})."
    ;;
  tamu)
    if [ "$(id -u)" -ne 0 ]; then
      echo "Run with sudo for TAMU: sudo env TAMU_IDENTITY=… TAMU_PASSWORD=… $0 tamu"
      exit 1
    fi
    if [ -z "${TAMU_IDENTITY:-}" ] || [ -z "${TAMU_PASSWORD:-}" ]; then
      echo "Set NetID credentials for WPA Enterprise (sudo does not pass your shell env by default):"
      echo "  sudo env TAMU_IDENTITY=your_netid TAMU_PASSWORD='your_password' $0 tamu"
      exit 1
    fi
    if [ -n "${TAMU_CA_CERT:-}" ] && [ ! -r "${TAMU_CA_CERT}" ]; then
      echo "TAMU_CA_CERT is set but not readable: ${TAMU_CA_CERT}"
      exit 1
    fi
    # TAMU KB528: NetID must be lowercase in identity.
    TAMU_IDENTITY="${TAMU_IDENTITY,,}"
    write_netplan_tamu > "$NETPLAN_OVERRIDE"
    chmod 600 "$NETPLAN_OVERRIDE"
    netplan_apply_quiet
    if ! patch_wpa_supplicant_for_tamu; then
      echo "TAMU wpa_supplicant patch failed. Try: sudo netplan --debug apply; sudo ls -la /run/netplan/wpa-wlan0.conf" >&2
      echo "Save WPA log: sudo journalctl -u netplan-wpa-wlan0.service -b --no-pager | tail -n 200 >\"${_WIFI_SCRIPT_DIR}/netplan-wpa-wlan0.log\"" >&2
      exit 1
    fi
    restart_netplan_wpa
    echo "Switched to TAMU (SSID ${TAMU_SSID}, DHCP). wpa_supplicant: peaplabel=0, proto=RSN, auth_alg=OPEN, eap_workaround=1, phase2 auth=MSCHAPV2."
    if [ -n "${TAMU_CA_CERT:-}" ]; then
      echo "Using ca-certificate: ${TAMU_CA_CERT}"
    else
      echo "No ca-certificate in netplan (optional). For USERTrust PEM: sudo env TAMU_CA_CERT=/etc/ssl/certs/USERTrust_RSA_Certification_Authority.pem … $0 tamu"
    fi
    if [ -n "${TAMU_PASSWORD_NT_HASH:-}" ]; then
      echo "Password sent as NT hash (TAMU_PASSWORD_NT_HASH)."
    fi
    ;;
  status)
    ssid=""
    iwgetid -r &>/dev/null && ssid=$(iwgetid -r)
    [ -z "$ssid" ] && ssid=$(wpa_cli -i wlan0 status 2>/dev/null | sed -n 's/^ssid=//p')
    if [ -n "$ssid" ]; then
      echo "SSID: $ssid"
    else
      echo "SSID: (unknown or no WiFi)"
    fi
    ip4=$(ip -4 -o addr show wlan0 2>/dev/null | awk '{print $4}' | head -1)
    if [ -n "$ip4" ]; then
      echo "wlan0 IP: $ip4"
    fi
    ;;
  *)
    usage
    ;;
esac
