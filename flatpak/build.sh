#!/usr/bin/env bash
# Build the Flatpak and install it for the current user.
#
# Usage:
#   bash flatpak/build.sh [--bundle]
#
#   --bundle   Also export a single-file .flatpak bundle to the project root.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
manifest="$here/io.github.mustek.A3IIIU2Tool.yml"
app_id="io.github.mustek.A3IIIU2Tool"
workdir="${TMPDIR:-/tmp}/mustek-flatpak"

if [ ! -d "$here/wheels" ]; then
    echo "==> Vendored wheels missing; fetching them"
    bash "$here/fetch-wheels.sh"
fi

echo "==> Building $app_id"
flatpak-builder --user --install --force-clean \
    --state-dir="$workdir/state" \
    --repo="$workdir/repo" \
    "$workdir/build" "$manifest"

echo "==> Installed $app_id (run: flatpak run $app_id)"

if [ "${1:-}" = "--bundle" ]; then
    out="$root/mustek-a3iiiu2-tool.flatpak"
    echo "==> Exporting bundle to $out"
    flatpak build-bundle "$workdir/repo" "$out" "$app_id"
    echo "==> Bundle ready: $out"
fi
