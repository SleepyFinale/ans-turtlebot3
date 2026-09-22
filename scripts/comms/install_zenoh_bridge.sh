#!/usr/bin/env bash
# Install zenoh-bridge-ros2dds for Ubuntu 22.04 (glibc 2.35) / Raspberry Pi.
#
# Do NOT use `apt install zenoh-bridge-ros2dds` on Jammy — that package needs
# libc6 >= 2.38 (Ubuntu 24.04).
#
# Strategy:
#   1) Prefer *-unknown-linux-gnu-standalone (glibc) and verify it runs.
#   2) Fall back to *-musl-standalone and install the `musl` package so
#      /lib/ld-musl-*.so.1 exists (plain musl binaries otherwise fail with
#      "No such file or directory" even when the file is present).
#
# Installs to: <workspace>/third_party/zenoh/zenoh-bridge-ros2dds
# Symlink:     ~/.local/bin/zenoh-bridge-ros2dds
#
# Use the SAME ZENOH_BRIDGE_VERSION on central and Clyde.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

ZENOH_BRIDGE_VERSION="${ZENOH_BRIDGE_VERSION:-1.7.2}"

ARCH="$(uname -m)"
case "$ARCH" in
  x86_64)  ARCH_TRIPLE_PREFIX="x86_64" ;;
  aarch64|arm64) ARCH_TRIPLE_PREFIX="aarch64" ;;
  *)
    echo "ERROR: unsupported architecture: $ARCH (need x86_64 or aarch64)"
    exit 1
    ;;
esac

DEST_DIR="${WORKSPACE_DIR}/third_party/zenoh"
STAGING="${DEST_DIR}/.staging-$$"
mkdir -p "$DEST_DIR" "$STAGING"
trap 'rm -rf "$STAGING"' EXIT

# unzip is often missing on minimal Pi images; prefer unzip, else Python.
extract_zip() {
  local zip_path="$1"
  local dest_dir="$2"
  if command -v unzip >/dev/null 2>&1; then
    unzip -q -o "$zip_path" -d "$dest_dir"
    return 0
  fi
  if command -v python3 >/dev/null 2>&1; then
    python3 - "$zip_path" "$dest_dir" <<'PY'
import sys, zipfile
from pathlib import Path
zf, dest = sys.argv[1], Path(sys.argv[2])
dest.mkdir(parents=True, exist_ok=True)
with zipfile.ZipFile(zf) as z:
    z.extractall(dest)
PY
    return 0
  fi
  echo "  ERROR: need 'unzip' or python3 to extract archives."
  echo "  Quick fix: sudo apt install -y unzip"
  return 1
}

download_and_extract() {
  local asset="$1"
  local urls=(
    "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_BRIDGE_VERSION}/${asset}"
    "https://download.eclipse.org/zenoh/zenoh-plugin-ros2dds/${ZENOH_BRIDGE_VERSION}/${asset}"
    "https://download.eclipse.org/zenoh/zenoh-plugin-ros2dds/latest/${asset}"
  )
  local zip_path="${STAGING}/${asset}"
  rm -rf "${STAGING}/unpacked"
  mkdir -p "${STAGING}/unpacked"

  local url downloaded=0
  for url in "${urls[@]}"; do
    echo "  trying: $url"
    if curl -fL --retry 3 --retry-delay 2 -o "$zip_path" "$url"; then
      downloaded=1
      break
    fi
    echo "  download failed, trying next mirror..."
  done
  if [[ "$downloaded" -ne 1 ]]; then
    return 1
  fi

  echo "  unpacking ${asset}..."
  if ! extract_zip "$zip_path" "${STAGING}/unpacked"; then
    return 1
  fi
  while IFS= read -r -d '' nested; do
    echo "  unpacking nested: $(basename "$nested")"
    extract_zip "$nested" "${STAGING}/unpacked" || return 1
  done < <(find "${STAGING}/unpacked" -type f -name 'zenoh-bridge-ros2dds*.zip' -print0 2>/dev/null || true)

  local bin
  bin="$(find "${STAGING}/unpacked" -type f -name 'zenoh-bridge-ros2dds' | head -n 1 || true)"
  if [[ -z "$bin" ]]; then
    echo "  ERROR: zenoh-bridge-ros2dds not found in archive"
    find "${STAGING}/unpacked" -maxdepth 3 -type f | sed 's/^/    /' || true
    return 1
  fi
  install -m 0755 "$bin" "${DEST_DIR}/zenoh-bridge-ros2dds"
  return 0
}

binary_runs() {
  local bin="$1"
  [[ -x "$bin" ]] || return 1
  # "No such file or directory" on a present ELF usually means missing interpreter.
  if ! "$bin" -h >/dev/null 2>&1 && ! "$bin" --help >/dev/null 2>&1; then
    # Some builds exit non-zero on -h but still execute; check loader error.
    local err
    err="$("$bin" -h 2>&1 || true)"
    if echo "$err" | grep -qi 'No such file or directory'; then
      return 1
    fi
    # If we got usage/help text, treat as success.
    if echo "$err" | grep -qiE 'usage|config|zenoh'; then
      return 0
    fi
    # Try running with --version-ish; if exec fails hard, return 1.
    if ! "$bin" -c /dev/null >/dev/null 2>&1; then
      # Still may be OK if config parse failed after successful exec.
      if echo "$("$bin" -c /dev/null 2>&1 || true)" | grep -qi 'No such file or directory'; then
        return 1
      fi
    fi
  fi
  return 0
}

ensure_musl_loader() {
  local musl_interp
  case "$ARCH" in
    x86_64) musl_interp="/lib/ld-musl-x86_64.so.1" ;;
    aarch64|arm64) musl_interp="/lib/ld-musl-aarch64.so.1" ;;
  esac
  if [[ -e "$musl_interp" ]]; then
    return 0
  fi
  echo "  musl loader missing ($musl_interp); installing package 'musl'..."
  if [[ "${EUID}" -eq 0 ]]; then
    apt-get update -qq
    apt-get install -y musl
  else
    sudo apt-get update -qq
    sudo apt-get install -y musl
  fi
}

echo "Installing zenoh-bridge-ros2dds ${ZENOH_BRIDGE_VERSION} for ${ARCH}"
echo "  workspace: ${WORKSPACE_DIR}"

INSTALLED_TARGET=""
GNU_ASSET="zenoh-plugin-ros2dds-${ZENOH_BRIDGE_VERSION}-${ARCH_TRIPLE_PREFIX}-unknown-linux-gnu-standalone.zip"
MUSL_ASSET="zenoh-plugin-ros2dds-${ZENOH_BRIDGE_VERSION}-${ARCH_TRIPLE_PREFIX}-unknown-linux-musl-standalone.zip"

echo "Attempt 1: glibc (gnu) standalone"
if download_and_extract "$GNU_ASSET"; then
  if binary_runs "${DEST_DIR}/zenoh-bridge-ros2dds"; then
    INSTALLED_TARGET="${ARCH_TRIPLE_PREFIX}-unknown-linux-gnu"
  else
    echo "  gnu binary present but does not run on this host (likely newer glibc)."
  fi
else
  echo "  gnu download failed."
fi

if [[ -z "$INSTALLED_TARGET" ]]; then
  echo "Attempt 2: musl standalone (+ musl loader package)"
  if download_and_extract "$MUSL_ASSET"; then
    ensure_musl_loader
    if binary_runs "${DEST_DIR}/zenoh-bridge-ros2dds"; then
      INSTALLED_TARGET="${ARCH_TRIPLE_PREFIX}-unknown-linux-musl"
    else
      echo "  musl binary still does not run after installing musl loader."
    fi
  else
    echo "  musl download failed."
  fi
fi

if [[ -z "$INSTALLED_TARGET" ]]; then
  echo "ERROR: could not install a runnable zenoh-bridge-ros2dds."
  echo "Place a working binary manually at: ${DEST_DIR}/zenoh-bridge-ros2dds"
  exit 1
fi

echo "${ZENOH_BRIDGE_VERSION}" >"${DEST_DIR}/VERSION"
echo "${INSTALLED_TARGET}" >"${DEST_DIR}/TARGET"

LOCAL_BIN="${HOME}/.local/bin"
mkdir -p "$LOCAL_BIN"
ln -sfn "${DEST_DIR}/zenoh-bridge-ros2dds" "${LOCAL_BIN}/zenoh-bridge-ros2dds"

echo ""
echo "Installed: ${DEST_DIR}/zenoh-bridge-ros2dds"
echo "Target:    ${INSTALLED_TARGET}"
echo "Symlink:   ${LOCAL_BIN}/zenoh-bridge-ros2dds"
echo ""
echo "Also ensure CycloneDDS RMW is installed:"
echo "  sudo apt install -y ros-humble-rmw-cyclonedds-cpp"
echo "Done."
