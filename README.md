# Mustek ScanExpress A3 USB PRO Tool

Command-line and graphical scanning tools for the **Mustek ScanExpress A3 USB
PRO** flatbed scanner on Linux.

The tool talks to the scanner directly over USB — no proprietary software or
kernel driver is required.

## Features

- Full A3 scanning over USB with a pure-Python driver
- Command-line tool (`mustek-cli`) and PyQt6 GUI (`mustek-gui`)
- Color (24/48-bit) and grayscale (8/16-bit) modes
- Resolution selection (150 / 300 / 600 / 1200 DPI, limited to the connected model)
- Adjustable scan window (width/height in mm) and top trim
- PNG, JPEG and multi-page TIFF output

## Requirements

- Python ≥ 3.9
- Linux with USB access
- `pyusb` and `Pillow` (installed automatically)
- `PyQt6` for the GUI (installed via the `gui` extra)

## Installation

```bash
# CLI only
pip install .

# CLI + GUI
pip install ".[gui]"

# Editable install for development
pip install -e ".[gui,dev]"
```

### USB permissions (udev)

The scanner uses USB `055F:040B`. Install the bundled udev rule so a non-root
user can access it:

```bash
sudo cp "$(python -c 'import mustek_driver, os; print(os.path.dirname(mustek_driver.__file__))')/udev/90-mustek.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and replug the scanner afterwards. Without the rule you must run as root.

## Flatpak (self-contained app)

A Flatpak build is provided under `flatpak/`. It bundles Python, PyQt6 and the
scanning driver into a single sandboxed app — the host only needs the
`org.freedesktop.Platform` runtime and Flatpak itself.

Build and install for the current user:

```bash
bash flatpak/build.sh              # build + install
bash flatpak/build.sh --bundle     # also write mustek-a3iiiu2-tool.flatpak
```

Run it:

```bash
flatpak run io.github.mustek.A3IIIU2Tool        # GUI
flatpak run --command=mustek-cli io.github.mustek.A3IIIU2Tool --help   # CLI
```

Or install a prebuilt single-file bundle:

```bash
flatpak install --user ./mustek-a3iiiu2-tool.flatpak
```

Notes:

- The app is granted raw USB access (`--device=all`) so it can talk to the
  scanner; the udev rule is **not** needed inside the Flatpak.
- `flatpak-builder` builds offline, so `flatpak/fetch-wheels.sh` vendors the
  required Python wheels into `flatpak/wheels/` before building (these are
  git-ignored). Requires network access at that step only.
- The `flatpak/` manifest uses the placeholder app ID
  `io.github.mustek.A3IIIU2Tool`; change it (and the desktop/metainfo filenames)
  to match your own reverse-DNS namespace before publishing.

## CLI usage

```
mustek-cli [options]
```

| Option | Default | Description |
|--------|---------|-------------|
| `--mode` | `color48` | Color depth: `gray8`, `gray16`, `color24`, `color48` |
| `--dpi` | `300` | Resolution: `150`, `300`, `600`, `1200` |
| `--out` | `mustek_{dpi}_{mode}.png` | Output path (`.png`, `.jpg`, `.tiff`) |
| `--width` | `297` | Scan window width in mm |
| `--height` | `420` | Scan window height in mm |
| `--trim-top` | `4` | Drop this many mm from the top of the scan |
| `--list` | | Probe the device and print its model resolution |
| `--dry-run` | | Open the scanner, print geometry, do not scan |
| `--verbose` | | Enable debug logging |
| `--quiet` | | Suppress warnings |

Examples:

```bash
# Scan A3 at 300 DPI in color (default)
mustek-cli

# Grayscale scan at 600 DPI
mustek-cli --mode gray8 --dpi 600 --out scan.png

# Check the connected scanner
mustek-cli --list
```

## GUI usage

```
mustek-gui
```

The GUI provides:

- Mode and resolution selection
- Scan window controls (width, height, top trim) with paper-format presets
- Live progress and an image preview
- Save (with a file dialog) and Quick Save +1 (auto-incrementing filename) —
  the last used directory and filename are remembered
- List Device to probe the connected scanner

GUI settings (mode, resolution, trim, format, window size, save location) are
preserved between runs.

## Scan modes

| Mode | CLI flag | Bits/pixel | Description |
|------|----------|------------|-------------|
| GRAY8 | `gray8` | 1 | 8-bit grayscale |
| GRAY16 | `gray16` | 2 | 16-bit grayscale |
| COLOR24 | `color24` | 3 | 24-bit RGB |
| COLOR48 | `color48` | 6 | 48-bit RGB |

The maximum resolution depends on the connected model (600 or 1200 DPI); higher
requests are clamped to the model maximum.

## Notes

- **1200 DPI banding** — at 1200 DPI the three-segment CCD shows per-segment gain
  imbalance; grayscale modes are recommended at that resolution.
- **Single scanner** — one scanner per process.

## License

MIT
