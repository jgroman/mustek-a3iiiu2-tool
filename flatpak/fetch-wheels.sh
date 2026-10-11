#!/usr/bin/env bash
# Download the Python wheels the Flatpak build installs. flatpak-builder builds
# offline, so the wheels must be vendored before building.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
dest="$here/wheels"

mkdir -p "$dest"
python3 -m pip download --only-binary=:all: --dest "$dest" \
    pyusb Pillow PyQt6

echo "Wheels written to $dest"
ls -1 "$dest"
