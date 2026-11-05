#!/bin/bash
#
# OS-level configuration every fleet robot needs and git does not carry:
# packages, services, kernel and journald settings, the Wi-Fi reconnect hook.
# Safe to re-run. docs/setup.md explains why each item is here.
#
# Usage (on the robot, logged in as the robot user):
#   sudo ./scripts/setup/robot_system_config.sh     # apply
#   ./scripts/setup/robot_system_config.sh --check  # report only, changes nothing
#

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

APT_PACKAGES=(
  # Stereo: cameras and the depth node
  v4l-utils ros-humble-v4l2-camera ros-humble-cv-bridge python3-opencv
  # DDS, Wi-Fi tuning, time sync
  ros-humble-rmw-cyclonedds-cpp iw chrony
  # Python tooling
  python3-pip python3.10-venv
  # Debug and capture tools
  ros-humble-foxglove-bridge ros-humble-image-transport-plugins ros-humble-usb-cam tshark
)
PIP_PACKAGES=(numpy==1.26.4 pyzmq==27.1.0)
# They ship to Logstash at 192.168.0.76:5044, unreachable from TAMU_WiFi: endless
# retries, ~470 MB RAM and ~12% CPU. `systemctl enable --now` them for security experiments.
DISABLED_SERVICES=(metricbeat packetbeat heartbeat-elastic bluetooth hciuart)
# Not used by the fleet, and two of them are daemons running all the time.
UNUSED_PACKAGES=(openvswitch-switch-dpdk openvswitch-switch openvswitch-common python3-openvswitch dpdk rdma-core)

WIFI_HOOK=/etc/networkd-dispatcher/routable.d/50-wifi-powersave-off
SYSCTL_CONF=/etc/sysctl.d/60-cyclonedds.conf
JOURNALD_CONF=/etc/systemd/journald.conf.d/size.conf
SUDOERS_CAPTURE=/etc/sudoers.d/packet-capture
STALE_BASHRC='turtlebot3/scripts/ros_(robot_env|domain_profile)\.bash'

failures=0
report() {  # report <ok|fail|warn> <text>
  case "$1" in
    ok) echo "  ok    $2" ;;
    warn) echo "  WARN  $2" ;;
    *) echo "  FAIL  $2"; failures=$((failures + 1)) ;;
  esac
}
expect() {  # expect <text> <command...>
  local text="$1"; shift
  if "$@" >/dev/null 2>&1; then report ok "$text"; else report fail "$text"; fi
}
service_off() {
  [ "$(systemctl is-enabled "$1" 2>/dev/null)" != enabled ] && ! systemctl is-active --quiet "$1"
}
package_absent() {
  [ "$(dpkg-query -W -f='${db:Status-Status}' "$1" 2>/dev/null)" != installed ]
}

run_check() {
  local user_name="${USER:-$(id -un)}"
  echo "Packages:"
  for pkg in "${APT_PACKAGES[@]}"; do
    if package_absent "$pkg"; then report fail "$pkg installed"; else report ok "$pkg installed"; fi
  done
  for pkg in "${UNUSED_PACKAGES[@]}"; do
    expect "$pkg absent" package_absent "$pkg"
  done
  for spec in "${PIP_PACKAGES[@]}"; do
    expect "pip --user $spec" python3 -m pip show --disable-pip-version-check "${spec%%==*}"
    [ "$(python3 -m pip show "${spec%%==*}" 2>/dev/null | awk '/^Version:/{print $2}')" = "${spec##*==}" ] \
      || report fail "pip version of ${spec%%==*} is ${spec##*==}"
  done

  echo "Services:"
  for svc in "${DISABLED_SERVICES[@]}"; do
    expect "$svc off" service_off "$svc"
  done
  expect "chrony running" systemctl is-active --quiet chrony
  expect "systemd-timesyncd off" service_off systemd-timesyncd
  expect "boot-wifi enabled" systemctl is-enabled --quiet boot-wifi
  expect "wifi-powersave-off enabled" systemctl is-enabled --quiet wifi-powersave-off

  echo "Settings:"
  expect "Wi-Fi reconnect hook executable" test -x "$WIFI_HOOK"
  expect "Wi-Fi power save off now" bash -c "iw dev wlan0 get power_save | grep -q 'Power save: off'"
  expect "net.core.rmem_max is 8388608" test "$(cat /proc/sys/net/core/rmem_max)" = 8388608
  expect "journald capped in $JOURNALD_CONF" grep -q '^SystemMaxUse=200M' "$JOURNALD_CONF"
  expect "sudo tcpdump and tshark without a password" bash -c \
    "sudo -n -l /usr/bin/tcpdump && sudo -n -l /usr/bin/tshark"

  echo "This account:"
  expect "~/.bashrc sources scripts/env/ros_robot_env.bash" grep -q 'scripts/env/ros_robot_env.bash' "$HOME/.bashrc"
  if grep -qE "$STALE_BASHRC" "$HOME/.bashrc"; then
    report fail "~/.bashrc still has the old scripts/ paths"
  else
    report ok "~/.bashrc has no old scripts/ paths"
  fi
  expect "tf2 overlay present (~/tf2_deadlock_fix)" test -f "$HOME/tf2_deadlock_fix/install/tf2/lib/libtf2.so"

  echo "Per-robot, set up by hand:"
  if [ -f "$WORKSPACE_DIR/config/stereo/$user_name.yaml" ]; then
    report ok "stereo calibration config/stereo/$user_name.yaml"
  else
    report warn "no config/stereo/$user_name.yaml: stereo needs a first calibration"
  fi
  if [ -f "$WORKSPACE_DIR/config/sensors/$user_name.yaml" ]; then
    report ok "hardware list config/sensors/$user_name.yaml"
  else
    report warn "no config/sensors/$user_name.yaml: defaults assume an ultrasonic sensor is fitted"
  fi

  echo
  if [ "$failures" -eq 0 ]; then
    echo "All checks passed."
  else
    echo "$failures check(s) failed."
    return 1
  fi
}

if [ "${1:-}" = "--check" ]; then
  if [ "$(id -u)" -eq 0 ]; then
    echo "Error: run --check as the robot user, not with sudo" >&2
    exit 2
  fi
  run_check
  exit $?
fi

set -e

if [ "$(id -u)" -ne 0 ]; then
  echo "Error: This script must be run as root (use sudo)"
  exit 1
fi
ROBOT_USER="${SUDO_USER:-}"
if [ -z "$ROBOT_USER" ] || [ "$ROBOT_USER" = root ]; then
  echo "Error: run it with sudo from the robot user's account"
  exit 1
fi
ROBOT_HOME="$(getent passwd "$ROBOT_USER" | cut -d: -f6)"

export DEBIAN_FRONTEND=noninteractive

echo "== Removing unused packages"
if grep -rqs openvswitch /etc/netplan; then
  echo "netplan refers to Open vSwitch: leaving it installed"
else
  to_remove=()
  for pkg in "${UNUSED_PACKAGES[@]}"; do
    package_absent "$pkg" || to_remove+=("$pkg")
  done
  if [ "${#to_remove[@]}" -gt 0 ]; then
    apt-get purge -y "${to_remove[@]}"
    apt-get autoremove -y --purge
  fi
fi

echo "== Installing packages"
echo 'wireshark-common wireshark-common/install-setuid boolean true' | debconf-set-selections
apt-get update -qq
apt-get install -y "${APT_PACKAGES[@]}"
sudo -u "$ROBOT_USER" -H python3 -m pip install --user --disable-pip-version-check "${PIP_PACKAGES[@]}"

echo "== Services"
for svc in "${DISABLED_SERVICES[@]}"; do
  if systemctl cat "$svc" >/dev/null 2>&1; then
    systemctl disable --now "$svc" 2>/dev/null || true
  fi
done
# Installing chrony removes timesyncd on current Ubuntu; an older chrony leaves it enabled.
if systemctl cat systemd-timesyncd >/dev/null 2>&1; then
  systemctl disable --now systemd-timesyncd 2>/dev/null || true
  systemctl mask systemd-timesyncd
fi

echo "== Wi-Fi power save hook"
cat > "$WIFI_HOOK" << 'EOF'
#!/bin/sh
# Keep Wi-Fi power save off after every (re)connect; the boot-time oneshot only covers the first link.
[ "$IFACE" = wlan0 ] && /usr/sbin/iw dev wlan0 set power_save off
exit 0
EOF
chmod 755 "$WIFI_HOOK"
iw dev wlan0 set power_save off || true

echo "== Kernel UDP buffers"
echo 'net.core.rmem_max=8388608' > "$SYSCTL_CONF"
sysctl -q -p "$SYSCTL_CONF"

echo "== Journald size cap"
mkdir -p "$(dirname "$JOURNALD_CONF")"
printf '[Journal]\nSystemMaxUse=200M\n' > "$JOURNALD_CONF"
systemctl restart systemd-journald
journalctl --vacuum-size=200M

echo "== Packet capture permissions"
tmp_sudoers="$(mktemp)"
echo "$ROBOT_USER ALL=(ALL) NOPASSWD: /usr/sbin/tcpdump, /usr/bin/tcpdump, /usr/bin/tshark" > "$tmp_sudoers"
visudo -cf "$tmp_sudoers" >/dev/null
install -m 0440 -o root -g root "$tmp_sudoers" "$SUDOERS_CAPTURE"
rm -f "$tmp_sudoers"
setcap cap_net_admin,cap_net_raw=eip "$(command -v tcpdump)"

echo "== ~/.bashrc paths"
sed -i -E 's#turtlebot3/scripts/(ros_robot_env|ros_domain_profile)\.bash#turtlebot3/scripts/env/\1.bash#g' \
  "$ROBOT_HOME/.bashrc"

if [ ! -f "$ROBOT_HOME/tf2_deadlock_fix/install/tf2/lib/libtf2.so" ]; then
  echo
  echo "Still to do by hand: copy the tf2 overlay from another robot (docs/setup.md)."
fi
echo
echo "Done. Reboot, then run without sudo: $0 --check"
