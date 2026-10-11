import argparse
import logging
import sys
from typing import Any, Callable, List, Optional

from mustek_driver import imageio, scanner_reflective
from mustek_driver.external import DriverError, configure_logging
from mustek_driver.scanner import Scanner
from mustek_driver.scanner_reflective import (
    SCAN_MODE_COLOR24, SCAN_MODE_COLOR48, SCAN_MODE_GRAY16, SCAN_MODE_GRAY8,
)
from mustek_driver.transport import LinuxUsbTransport

_MODES = ("gray8", "gray16", "color24", "color48")
_DPIS = (150, 300, 600, 1200)

# CLI mode -> public w_scan_mode for set_parameters
_PUBLIC_MODE = {
    "gray8": 1,
    "gray16": 9,
    "color24": 2,
    "color48": 10,
}

_SCAN_CONST = {
    "gray8": SCAN_MODE_GRAY8,
    "gray16": SCAN_MODE_GRAY16,
    "color24": SCAN_MODE_COLOR24,
    "color48": SCAN_MODE_COLOR48,
}


def _public_mode(mode: str) -> int:
    """CLI mode -> public w_scan_mode number for set_parameters."""
    return _PUBLIC_MODE[mode]


def _scan_constant(mode: str) -> int:
    """CLI mode -> SCAN_MODE_* constant (for imageio save)."""
    return _SCAN_CONST[mode]


def _scan_lines(height_mm: int, dpi: int) -> int:
    return int(height_mm * dpi / 25.4)


def _output_width(bpl: int, scan_constant: int) -> int:
    """Image width = bytes-per-line / bytes-per-pixel."""
    return bpl // imageio._bytes_per_pixel(scan_constant)


def _trim(data: bytes, bpl: int, trim_top: int) -> bytes:
    """Drop the first trim_top scan lines (post-scan top crop)."""
    if trim_top <= 0:
        return data
    return data[trim_top * bpl:]


def parse_args(argv: Optional[List[str]]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        prog="mustek-cli",
        description="Scan with a Mustek ScanExpress A3 USB PRO flatbed scanner.")
    ap.add_argument("--mode", choices=_MODES, default="color48",
                    help="color depth (default: color48)")
    ap.add_argument("--dpi", type=int, choices=_DPIS, default=300,
                    help="resolution (default: 300; 1200 has known banding)")
    ap.add_argument("--out", default=None, help="output path (.png/.jpg/.tiff)")
    ap.add_argument("--width", type=int, default=297,
                    help="scan window width in mm (default: 297)")
    ap.add_argument("--height", type=int, default=420,
                    help="scan window height in mm (default: 420)")
    ap.add_argument("--trim-top", type=int, default=4,
                    help="drop this many mm from the top (dark frame strip; "
                         "0 disables; default: 4)")
    ap.add_argument("--list", action="store_true",
                    help="probe for the device, print presence + model, exit")
    ap.add_argument("--dry-run", action="store_true",
                    help="open/power/set params, print geometry, do not scan")
    ap.add_argument("--verbose", action="store_true",
                    help="enable driver DEBUG logging")
    ap.add_argument("--quiet", action="store_true",
                    help="suppress the 1200-dpi banding warning")
    args = ap.parse_args(argv)
    if args.out is None:
        args.out = f"mustek_{args.dpi}_{args.mode}.png"
    return args


def _default_scanner() -> Scanner:
    return Scanner(LinuxUsbTransport())


def _run_scan(scanner: Scanner, args: argparse.Namespace) -> str:
    """Compose the driver scan (public API only) and return the saved path."""
    scanner.open()
    scanner.turn_power(1)
    scanner.determine_model_type()
    if args.verbose:
        configure_logging(logging.DEBUG)
    max_scan_dpi = max(scanner_reflective._DPI_TABLE_70)
    if args.dpi > max_scan_dpi:
        if not args.quiet:
            print(f"note: {args.dpi} dpi exceeds this model's max ({max_scan_dpi} "
                  "dpi); clamping", file=sys.stderr)
        args.dpi = max_scan_dpi
    if args.dpi >= 1200 and not args.quiet:
        print("warning: at 1200 dpi color/gray show known banding/darkness "
              "(per-CCD-segment imbalance)", file=sys.stderr)
    scanner.set_parameters({
        "w_scan_mode": _public_mode(args.mode),
        "w_dpi": args.dpi,
        "w_x1": 0, "w_y1": 0,
        "w_x2": args.width, "w_y2": args.height,
    })
    scanner.start_scan()
    try:
        total = _scan_lines(args.height, args.dpi)
        data = scanner.read_scanned_data(total)
        scan_const = _scan_constant(args.mode)
        bpl = scanner._reflective.dw_segment_bytes
        trim_lines = _scan_lines(args.trim_top, args.dpi)
        data = _trim(data, bpl, trim_lines)
        width = _output_width(bpl, scan_const)
        imageio.save_image(args.out, bytearray(data), scan_const, width,
                           args.dpi)
        return str(args.out)
    finally:
        scanner.stop_scan()


def _list(scanner: Scanner) -> int:
    scanner.open()
    scanner.turn_power(1)
    scanner.determine_model_type()
    try:
        max_scan_dpi = max(scanner_reflective._DPI_TABLE_70)
        print(f"Mustek scanner present (model {max_scan_dpi} DPI)")
        return 0
    finally:
        scanner.close()


def _dry_run(scanner: Scanner, args: argparse.Namespace) -> int:
    """Open/power/set params and print geometry, but do not start the scan."""
    scanner.open()
    scanner.turn_power(1)
    scanner.determine_model_type()
    if args.verbose:
        configure_logging(logging.DEBUG)
    max_scan_dpi = max(scanner_reflective._DPI_TABLE_70)
    if args.dpi > max_scan_dpi:
        if not args.quiet:
            print(f"note: {args.dpi} dpi exceeds this model's max ({max_scan_dpi} "
                  "dpi); clamping", file=sys.stderr)
        args.dpi = max_scan_dpi
    if args.dpi >= 1200 and not args.quiet:
        print("warning: at 1200 dpi color/gray show known banding/darkness "
              "(per-CCD-segment imbalance)", file=sys.stderr)
    scan_const = _scan_constant(args.mode)
    scanner.set_parameters({
        "w_scan_mode": _public_mode(args.mode),
        "w_dpi": args.dpi,
        "w_x1": 0, "w_y1": 0,
        "w_x2": args.width, "w_y2": args.height,
    })
    bpl = scanner._reflective.dw_segment_bytes
    width = _output_width(bpl, scan_const)
    height = _scan_lines(args.height, args.dpi)
    print(f"{width} x {height} px, {bpl} bytes/line")
    return 0


def main(argv: Optional[List[str]] = None,
         scanner_factory: Optional[Callable[[], Scanner]] = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    factory = scanner_factory if scanner_factory is not None else _default_scanner
    if args.list:
        try:
            return _list(factory())
        except (PermissionError, OSError, DriverError) as e:
            print(str(e), file=sys.stderr)
            return 1
    scanner = factory()
    try:
        if args.dry_run:
            return _dry_run(scanner, args)
        _run_scan(scanner, args)
        print(f"Wrote {args.out}")
        return 0
    except PermissionError as e:
        print("Cannot open scanner device. Install the udev rule "
              "(mustek_driver/udev/90-mustek.rules) and replug, or run as "
              "root. Details: %s" % e, file=sys.stderr)
        return 1
    except (DriverError, OSError, ValueError) as e:
        print(f"Scan failed: {e}", file=sys.stderr)
        return 1
    finally:
        try:
            scanner.close()
        except Exception:
            pass