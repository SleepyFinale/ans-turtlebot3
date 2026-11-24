#!/bin/bash
#
# Switch Raspberry Pi WiFi between Azure (hotspot) and TAMU_WiFi (campus).
#
# Usage:
#   sudo ./scripts/network/switch_wifi.sh azure     # Azure hotspot with static IP (per robot/user)
#   sudo ./scripts/network/switch_wifi.sh tamu      # TAMU_WiFi (DHCP; EAP-TLS if a JoinNow cert is installed, else PEAP)
#   ./scripts/network/switch_wifi.sh status         # show current WiFi (no sudo)
#
# Prereq: Remove or comment out the wifis/wlan0 block from
#   /etc/netplan/50-cloud-init.yaml so this script's 99-wifi-switch.yaml
#   is the only WiFi config (avoids "Duplicate access point SSID").
#
# Static IPs are chosen by current user (blinky / pinky / inky / clyde):
#   azure:  blinky@172.20.10.13, pinky@172.20.10.14, inky@172.20.10.15, clyde@172.20.10.16
#   tamu:   TAMU_WiFi (DHCP). EAP-TLS when $TAMU_TLS_DIR/client.crt and client.key exist
#           (SecureW2 JoinNow certificate), otherwise PEAP + wpa patch per TAMU Pi KB.
#           Force a method with TAMU_EAP=tls or TAMU_EAP=peap.
#           PEAP: identity schen08 (TAMU_IDENTITY); password from TAMU_PASSWORD or
#           /etc/wpa_supplicant/tamu/peap-password (root-only; never commit it)
#
#   Optional: TAMU_CA_CERT=… (PEM) to validate RADIUS cert; TAMU_PASSWORD_NT_HASH=… for password=hash:… (KB 528)
#             TAMU_DOMAIN_SUFFIX=tamu.edu only if Help Desk says your cert needs domain_suffix_match
#   Debug:  Save WPA logs next to this script - use _WIFI_SCRIPT_DIR printed by "tamu" errors, or:
#           sudo journalctl -u netplan-wpa-wlan0.service -b --no-pager | tail -n 200 > scripts/network/netplan-wpa-wlan0.log
# When run with sudo we use SUDO_USER so "blinky" user gets the blinky IPs.
# Override with: $0 azure blinky or ROBOT_NAME=….
#

set -e
NETPLAN_OVERRIDE="/etc/netplan/99-wifi-switch.yaml"
_WIFI_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Azure hotspot WiFi (azure) static IP config
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
TAMU_IDENTITY="${TAMU_IDENTITY:-schen08}"
# PEAP password: never in git. From TAMU_PASSWORD, else the first line of a root-only file on the robot.
TAMU_PASSWORD_FILE="${TAMU_PASSWORD_FILE:-/etc/wpa_supplicant/tamu/peap-password}"
# Optional PEM for ca-certificate (unset = none, matching TAMU Pi KB 528). Example: USERTrust under /etc/ssl/certs.
TAMU_CA_CERT="${TAMU_CA_CERT:-}"
# EAP-TLS (SecureW2 JoinNow certificate). Files, root-only: client.crt, client.key (unencrypted PEM),
# ca-bundle.pem. The EAP identity is the certificate CN (or the first line of $TAMU_TLS_DIR/identity).
# The server-name check (domain_suffix_match=net.tamu.edu) is added by /usr/local/sbin/netplan-wpa-fixups,
# run as ExecStartPre of netplan-wpa-wlan0.service (installed by scripts/setup/robot_system_config.sh).
TAMU_TLS_DIR="${TAMU_TLS_DIR:-/etc/wpa_supplicant/tamu}"
TAMU_EAP="${TAMU_EAP:-}"

# Resolve robot name: current user (when using sudo we use SUDO_USER), or ROBOT_NAME env or second argument
get_robot_name() {
  local name
  name="${SUDO_USER:-$USER}"
  [ -n "$name" ] && name=$(echo "$name" | tr '[:upper:]' '[:lower:]')
  echo "${name:-}"
}

# Set per-robot static IP for Azure. Exits if unknown robot.
set_robot_static_ips() {
  local robot
  robot="${1:-$(get_robot_name)}"
  robot="${robot,,}"
  case "$robot" in
    blinky)
      AZURE_STATIC_IP="$AZURE_IP_BLINKY"
      ;;
    pinky)
      AZURE_STATIC_IP="$AZURE_IP_PINKY"
      ;;
    inky)
      AZURE_STATIC_IP="$AZURE_IP_INKY"
      ;;
    clyde)
      AZURE_STATIC_IP="$AZURE_IP_CLYDE"
      ;;
    *)
      echo "Unknown robot: '$robot'. Current user is: $(get_robot_name)"
      echo "Use: $0 azure {blinky|pinky|inky|clyde}   (or set ROBOT_NAME=…)"
      exit 1
      ;;
  esac
}

usage() {
  echo "Usage: $0 { azure | tamu | status } [robot]"
  echo "  azure [blinky|pinky|inky|clyde] - connect to Azure hotspot (static IP by robot)"
  echo "  tamu                            - connect to TAMU_WiFi (DHCP; EAP-TLS if cert installed, else PEAP)"
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

# EAP-TLS when the JoinNow cert is installed (or TAMU_EAP=tls); PEAP otherwise (or TAMU_EAP=peap).
tamu_use_tls() {
  case "${TAMU_EAP,,}" in
    tls) return 0 ;;
    peap) return 1 ;;
  esac
  [ -r "${TAMU_TLS_DIR}/client.crt" ] && [ -r "${TAMU_TLS_DIR}/client.key" ]
}

tamu_tls_identity() {
  local identity
  identity="$(head -n1 "${TAMU_TLS_DIR}/identity" 2>/dev/null | tr -d '[:space:]')"
  if [ -z "$identity" ]; then
    # JoinNow uses the certificate CN as the EAP identity (e.g. 123456789@tamu.edu).
    identity="$(openssl x509 -in "${TAMU_TLS_DIR}/client.crt" -noout -subject -nameopt multiline 2>/dev/null \
      | sed -n 's/^ *commonName *= *//p' | head -n1)"
  fi
  echo "$identity"
}

write_netplan_tamu_tls() {
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
            method: tls
            identity: "$1"
            ca-certificate: "${TAMU_TLS_DIR}/ca-bundle.pem"
            client-certificate: "${TAMU_TLS_DIR}/client.crt"
            client-key: "${TAMU_TLS_DIR}/client.key"
EOF
}

case "${1:-}" in
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
      echo "Run with sudo for TAMU: sudo $0 tamu"
      exit 1
    fi
    if tamu_use_tls; then
      if [ ! -r "${TAMU_TLS_DIR}/ca-bundle.pem" ]; then
        echo "EAP-TLS needs ${TAMU_TLS_DIR}/ca-bundle.pem (TAMU RADIUS CA bundle from JoinNow)." >&2
        exit 1
      fi
      tls_identity="$(tamu_tls_identity)"
      if [ -z "$tls_identity" ]; then
        echo "Could not determine the EAP-TLS identity; write it to ${TAMU_TLS_DIR}/identity." >&2
        exit 1
      fi
      write_netplan_tamu_tls "$tls_identity" > "$NETPLAN_OVERRIDE"
      chmod 600 "$NETPLAN_OVERRIDE"
      if [ ! -x /usr/local/sbin/netplan-wpa-fixups ]; then
        echo "Warning: /usr/local/sbin/netplan-wpa-fixups missing (run scripts/setup/robot_system_config.sh);" \
          "RADIUS server name is not checked." >&2
      fi
      # netplan apply (re)starts netplan-wpa-wlan0, whose ExecStartPre adds domain_suffix_match.
      # No second restart: stopping wpa_supplicant while brcmfmac is still creating p2p-dev-wlan0
      # oopsed the driver at boot (2026-11-24) and hung boot-wifi.service.
      netplan_apply_quiet
      echo "Switched to TAMU (SSID ${TAMU_SSID}, DHCP, EAP-TLS, identity ${tls_identity})."
      exit 0
    fi
    if [ -n "${TAMU_CA_CERT:-}" ] && [ ! -r "${TAMU_CA_CERT}" ]; then
      echo "TAMU_CA_CERT is set but not readable: ${TAMU_CA_CERT}"
      exit 1
    fi
    if [ -z "${TAMU_PASSWORD:-}" ] && [ -r "$TAMU_PASSWORD_FILE" ]; then
      TAMU_PASSWORD="$(head -n1 "$TAMU_PASSWORD_FILE")"
    fi
    if [ -z "${TAMU_PASSWORD:-}" ]; then
      echo "No TAMU EAP-TLS certificate in ${TAMU_TLS_DIR} and no PEAP password." >&2
      echo "Install the certificate (docs/setup.md, \"TAMU_WiFi certificate (EAP-TLS)\"), or put the NetID" >&2
      echo "password in ${TAMU_PASSWORD_FILE} (root:root, mode 600), or set TAMU_PASSWORD." >&2
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
    echo "Switched to TAMU (SSID ${TAMU_SSID}, DHCP, identity ${TAMU_IDENTITY}). wpa_supplicant: peaplabel=0, proto=RSN, auth_alg=OPEN, eap_workaround=1, phase2 auth=MSCHAPV2."
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
