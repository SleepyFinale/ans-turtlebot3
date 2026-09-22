#!/usr/bin/env bash
# Install zenoh-bridge-ros2dds for Ubuntu 22.04 (glibc 2.35) / Raspberry Pi OS.
#
# The Eclipse apt package currently depends on libc6 >= 2.38 (Ubuntu 24.04+) and
# fails on Jammy. This script downloads a pinned musl standalone binary instead
# (no newer glibc required) into:
#   <workspace>/third_party/zenoh/zenoh-bridge-ros2dds
# and optionally symlinks into ~/.local/bin.
#
# Run on central and on Clyde. Use the SAME ZENOH_BRIDGE_VERSION on both.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# Pin so central and robots stay on matching protocol versions.
ZENOH_BRIDGE_VERSION="${ZENOH_BRIDGE_VERSION:-1.7.2}"

ARCH="$(uname -m)"
case "$ARCH" in
  x86_64)
    TARGET="x86_64-unknown-linux-musl"
    ;;
  aarch64|arm64)
    TARGET="aarch64-unknown-linux-musl"
    ;;
  *)
    echo "ERROR: unsupported architecture: $ARCH"
    echo "Supported: x86_64 (central), aarch64 (Pi)."
    exit 1
    ;;
esac

ASSET="zenoh-plugin-ros2dds-${ZENOH_BRIDGE_VERSION}-${TARGET}-standalone.zip"
# Prefer GitHub release assets; Eclipse download mirror as fallback.
URLS=(
  "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_BRIDGE_VERSION}/${ASSET}"
  "https://download.eclipse.org/zenoh/zenoh-plugin-ros2dds/${ZENOH_BRIDGE_VERSION}/${ASSET}"
  "https://download.eclipse.org/zenoh/zenoh-plugin-ros2dds/latest/${ASSET}"
)

DEST_DIR="${WORKSPACE_DIR}/third_party/zenoh"
STAGING="${DEST_DIR}/.staging-$$"
mkdir -p "$DEST_DIR" "$STAGING"
trap 'rm -rf "$STAGING"' EXIT

ZIP_PATH="${STAGING}/${ASSET}"
echo "Installing zenoh-bridge-ros2dds ${ZENOH_BRIDGE_VERSION} (${TARGET})"
echo "  workspace: ${WORKSPACE_DIR}"

downloaded=0
for url in "${URLS[@]}"; do
  echo "  trying: $url"
  if curl -fL --retry 3 --retry-delay 2 -o "$ZIP_PATH" "$url"; then
    downloaded=1
    break
  fi
  echo "  download failed, trying next mirror..."
done

if [[ "$downloaded" -ne 1 ]]; then
  echo "ERROR: could not download ${ASSET}"
  echo "Download manually and place the zenoh-bridge-ros2dds binary at:"
  echo "  ${DEST_DIR}/zenoh-bridge-ros2dds"
  exit 1
fi

echo "  unpacking..."
unzip -q -o "$ZIP_PATH" -d "$STAGING/unpacked"

# Standalone archives sometimes nest another zip for the bridge.
while IFS= read -r -d '' nested; do
  echo "  unpacking nested: $(basename "$nested")"
  unzip -q -o "$nested" -d "$STAGING/unpacked"
done < <(find "$STAGING/unpacked" -type f -name 'zenoh-bridge-ros2dds*.zip' -print0 2>/dev/null || true)

BIN="$(find "$STAGING/unpacked" -type f -name 'zenoh-bridge-ros2dds' | head -n 1 || true)"
if [[ -z "$BIN" ]]; then
  echo "ERROR: zenoh-bridge-ros2dds binary not found inside ${ASSET}"
  echo "Archive contents:"
  find "$STAGING/unpacked" -maxdepth 3 -type f | sed 's/^/  /'
  exit 1
fi

install -m 0755 "$BIN" "${DEST_DIR}/zenoh-bridge-ros2dds"
echo "${ZENOH_BRIDGE_VERSION}" >"${DEST_DIR}/VERSION"
echo "${TARGET}" >"${DEST_DIR}/TARGET"

# Help interactive shells find it without editing start scripts.
LOCAL_BIN="${HOME}/.local/bin"
mkdir -p "$LOCAL_BIN"
ln -sfn "${DEST_DIR}/zenoh-bridge-ros2dds" "${LOCAL_BIN}/zenoh-bridge-ros2dds"

echo ""
echo "Installed: ${DEST_DIR}/zenoh-bridge-ros2dds"
echo "Symlink:   ${LOCAL_BIN}/zenoh-bridge-ros2dds"
"${DEST_DIR}/zenoh-bridge-ros2dds" -h >/dev/null 2>&1 \
  || "${DEST_DIR}/zenoh-bridge-ros2dds" --help >/dev/null 2>&1 \
  || true

echo ""
echo "Also ensure CycloneDDS RMW is installed:"
echo "  sudo apt install ros-humble-rmw-cyclonedds-cpp"
echo ""
echo "If '${LOCAL_BIN}' is not on PATH, either open a new shell or run:"
echo "  export PATH=\"${LOCAL_BIN}:\$PATH\""
echo "Done."
