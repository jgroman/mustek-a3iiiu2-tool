"""Save scanned pixel data to image files (PNG, JPEG, multi-page TIFF).

Uses Pillow for all image I/O. Pillow is a required dependency.
"""
from pathlib import Path
from typing import List, Union

from PIL import Image

from mustek_driver.scanner_reflective import (
    SCAN_MODE_GRAY16, SCAN_MODE_GRAY8, SCAN_MODE_TEXT8,
    SCAN_MODE_COLOR48, SCAN_MODE_COLOR24,
)

_MODE_MAP = {
    SCAN_MODE_GRAY16: "I;16",
    SCAN_MODE_GRAY8: "L",
    SCAN_MODE_TEXT8: "L",
    SCAN_MODE_COLOR48: "RGB",
    SCAN_MODE_COLOR24: "RGB",
}

_FORMAT_NAMES = {
    ".png": "PNG",
    ".jpg": "JPEG",
    ".jpeg": "JPEG",
    ".tiff": "TIFF",
    ".tif": "TIFF",
}

PageData = Union[bytearray, bytes]


def _bytes_per_pixel(scan_mode: int) -> int:
    """Return bytes per pixel for the given scan_mode."""
    if scan_mode in (SCAN_MODE_GRAY16,):
        return 2
    if scan_mode in (SCAN_MODE_GRAY8, SCAN_MODE_TEXT8):
        return 1
    if scan_mode == SCAN_MODE_COLOR48:
        return 6
    if scan_mode == SCAN_MODE_COLOR24:
        return 3
    raise ValueError(f"Unknown scan_mode: {scan_mode}")


def save_image(path, lines: Union[PageData, List[PageData]], scan_mode: int,
               width: int, dpi: int, *, swap_rgb: bool = False) -> None:
    """Save scanned pixel data to an image file.

    Args:
        path: Output file path. Extension determines format (.png, .jpg, .tiff).
        lines: Pixel data — bytearray for single page, list[bytearray] for
               multi-page (TIFF only). Each bytearray is a complete page's
               pixel data as returned by read_scanned_data().
        scan_mode: One of the SCAN_MODE_* constants from scanner_reflective.
        width: Pixel width of each line (n_segment_pixels).
        dpi: Scan resolution (stored in TIFF/PNG metadata).
        swap_rgb: If True, swap R and B channels (currently unused, reserved).
    """
    path = Path(path)
    fmt_name = _FORMAT_NAMES.get(path.suffix.lower())
    if fmt_name is None:
        raise ValueError(
            f"Unsupported format: {path.suffix!r}. "
            f"Use one of: {', '.join(_FORMAT_NAMES.keys())}")

    # Normalize lines to list of pages
    pages: List[PageData]
    if isinstance(lines, bytearray):
        pages = [lines]
    elif isinstance(lines, list):
        if not lines:
            raise ValueError("No pixel data")
        pages = lines
    else:
        raise TypeError(
            f"lines must be bytearray or list[bytearray], "
            f"got {type(lines).__name__}")

    # Validate multi-page
    if len(pages) > 1 and fmt_name != "TIFF":
        raise ValueError("Multi-page only supported for TIFF")

    # Map scan_mode to Pillow mode
    pil_mode = _MODE_MAP.get(scan_mode)
    if pil_mode is None:
        raise ValueError(f"Unknown scan_mode: {scan_mode}")

    bpp = _bytes_per_pixel(scan_mode)

    # JPEG rejects 16-bit
    if fmt_name == "JPEG" and scan_mode in (SCAN_MODE_GRAY16, SCAN_MODE_COLOR48):
        raise ValueError(
            "JPEG does not support 16-bit; use TIFF or PNG")

    # Build pages as PIL Images
    images = []
    for page_data in pages:
        if not page_data:
            raise ValueError("No pixel data")
        n_pixels = len(page_data) // bpp
        height = n_pixels // width
        if scan_mode == SCAN_MODE_COLOR48:
            # 6 bytes/pixel (16-bit R/G/B). Pillow "RGB" is 8-bit, so down-convert
            # each 16-bit channel to its high byte (>>8) first — otherwise the
            # 6-byte data is misinterpreted as 3-byte RGB and the image is dark.
            out = bytearray(n_pixels * 3)
            for i in range(n_pixels):
                o = i * 6
                out[i * 3] = page_data[o + 1]
                out[i * 3 + 1] = page_data[o + 3]
                out[i * 3 + 2] = page_data[o + 5]
            img = Image.frombytes("RGB", (width, height), bytes(out))
        else:
            img = Image.frombytes(pil_mode, (width, height), bytes(page_data))
        images.append(img)

    # Save
    first = images[0]
    if fmt_name == "TIFF" and len(images) > 1:
        first.save(path, format=fmt_name, save_all=True,
                   append_images=images[1:], dpi=(dpi, dpi))
    else:
        first.save(path, format=fmt_name, dpi=(dpi, dpi))
