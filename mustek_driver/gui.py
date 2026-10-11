"""Scanning GUI for the Mustek ScanExpress A3 USB PRO scanner.

Provides a PyQt6 window with mode/DPI/geometry controls, live progress,
and image preview. Reuses cli.py pure helpers.
"""
import os
import re
import sys
from typing import Callable, List, Optional

from PyQt6.QtCore import Qt, QThread, pyqtSignal
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QComboBox, QSpinBox, QLabel, QPushButton, QProgressBar,
    QFileDialog, QMessageBox, QScrollArea, QFrame, QSizePolicy,
)
from PyQt6.QtCore import QSettings
from PyQt6.QtGui import QImage, QPixmap

from mustek_driver import scanner_reflective
from mustek_driver.scanner_reflective import (
    SCAN_MODE_GRAY16, SCAN_MODE_GRAY8, SCAN_MODE_TEXT8,
    SCAN_MODE_COLOR24, SCAN_MODE_COLOR48,
)
from mustek_driver.external import DriverError
from mustek_driver.cli import (
    _public_mode, _scan_constant, _scan_lines, _trim, _output_width,
)
from mustek_driver.scanner import Scanner
from mustek_driver.transport import LinuxUsbTransport

_QFORMAT = {
    SCAN_MODE_GRAY8: QImage.Format.Format_Grayscale8,
    SCAN_MODE_TEXT8: QImage.Format.Format_Grayscale8,
    SCAN_MODE_COLOR24: QImage.Format.Format_RGB888,
}


def _data_to_qimage(data: bytes, scan_mode: int, width: int, height: int) -> QImage:
    """Convert raw scan bytes to a QImage for display.

    GRAY16 and COLOR48 are down-converted to 8-bit (high byte) since
    QImage has no native 16-bit grayscale or 48-bit RGB format.

    All QImage constructions pass an explicit bytesPerLine: Qt otherwise pads
    each scanline to a 4-byte boundary (QImage(data, w, h, fmt) computes
    bytesPerLine = (w*bpp + 3) & ~3), but our pixel data is tightly packed.
    When w*bpp is not a multiple of 4, the default constructor would read each
    subsequent row 1 pixel late, producing the cumulative 1px-left shear seen
    at full-width 300-dpi (3507px*3bpp = 10521, not 4-aligned) but not at
    A4-portrait (2480px*3bpp = 7440, 4-aligned).
    """
    if not data:
        raise ValueError("No pixel data")

    if scan_mode in (SCAN_MODE_GRAY8, SCAN_MODE_TEXT8, SCAN_MODE_COLOR24):
        fmt = _QFORMAT[scan_mode]
        bpp = 1 if scan_mode in (SCAN_MODE_GRAY8, SCAN_MODE_TEXT8) else 3
        expected = width * height * bpp
        if len(data) < expected:
            raise ValueError(f"Data too short: {len(data)} < {expected}")
        # QImage wraps the buffer — caller must keep data alive
        return QImage(bytes(data), width, height, width * bpp, fmt)

    if scan_mode == SCAN_MODE_GRAY16:
        # Down-convert 16-bit → 8-bit (high byte)
        expected = width * height * 2
        if len(data) < expected:
            raise ValueError(f"Data too short: {len(data)} < {expected}")
        out = bytearray(width * height)
        for i in range(width * height):
            out[i] = data[i * 2 + 1]  # high byte
        return QImage(bytes(out), width, height, width,
                      QImage.Format.Format_Grayscale8)

    if scan_mode == SCAN_MODE_COLOR48:
        # Down-convert 6-byte RGB → 3-byte RGB (high byte of each channel)
        n_pixels = width * height
        expected = n_pixels * 6
        if len(data) < expected:
            raise ValueError(f"Data too short: {len(data)} < {expected}")
        out = bytearray(n_pixels * 3)
        for i in range(n_pixels):
            o = i * 6
            out[i * 3] = data[o + 1]      # R high
            out[i * 3 + 1] = data[o + 3]  # G high
            out[i * 3 + 2] = data[o + 5]  # B high
        return QImage(bytes(out), width, height, width * 3,
                      QImage.Format.Format_RGB888)

    raise ValueError(f"Unknown scan_mode: {scan_mode}")


# Progress chunk size — emit progress every N lines
_PROGRESS_CHUNK = 200


class ScanThread(QThread):
    """Background thread that performs a scan and emits results."""

    progress = pyqtSignal(int, str)     # (lines_done, status_text)
    finished = pyqtSignal(object)       # QImage
    error = pyqtSignal(str)             # error message

    def __init__(self, mode: str, dpi: int, width: int, height: int,
                 trim_top_mm: int, scanner_factory: Callable[[], Scanner],
                 parent=None) -> None:
        super().__init__(parent)
        self._mode = mode
        self._dpi = dpi
        self._width = width
        self._height = height
        self._trim_top_mm = trim_top_mm
        self._factory = scanner_factory

    def run(self) -> None:
        try:
            self._do_scan()
        except Exception as e:
            self.error.emit(str(e))

    def _do_scan(self) -> None:
        scanner = self._factory()
        try:
            scanner.open()
            scanner.turn_power(1)
            scanner.determine_model_type()
            max_scan_dpi = max(scanner_reflective._DPI_TABLE_70)
            dpi = min(self._dpi, max_scan_dpi)
            scanner.set_parameters({
                "w_scan_mode": _public_mode(self._mode),
                "w_dpi": dpi,
                "w_x1": 0, "w_y1": 0,
                "w_x2": self._width, "w_y2": self._height,
            })
            scanner.start_scan()
            try:
                total = _scan_lines(self._height, dpi)
                bpl = scanner._reflective.dw_segment_bytes
                data = b""
                while len(data) // bpl < total:
                    remaining = total - len(data) // bpl
                    chunk = min(_PROGRESS_CHUNK, remaining)
                    data += scanner.read_scanned_data(chunk)
                    done = min(len(data) // bpl, total)
                    self.progress.emit(done, f"Scanning {done}/{total} lines")
                scan_const = _scan_constant(self._mode)
                trim_lines = _scan_lines(self._trim_top_mm, dpi)
                data = _trim(data, bpl, trim_lines)
                w = _output_width(bpl, scan_const)
                h = len(data) // bpl
                img = _data_to_qimage(data, scan_const, w, h)
                self.finished.emit(img)
            finally:
                scanner.stop_scan()
        finally:
            try:
                scanner.close()
            except Exception:
                pass


_MODES = ("gray8", "gray16", "color24", "color48")
_DPIS = (150, 300, 600, 1200)


def _increment_name(name: str) -> str:
    """Return the next sequential filename for Quick Save +1.

    Splits the trailing digits off the stem (before the last dot). If present,
    increments them keeping their width (leading zeros preserved); if that
    would overflow the width (e.g. 999 -> 1000) the full wider number is used.
    If the stem ends without a number, appends "1".

    Examples: picture.png -> picture1.png, picture9.png -> picture10.png,
    picture001.png -> picture002.png, picture999.png -> picture1000.png.
    """
    stem, dot, ext = name.rpartition(".")
    if not dot:
        stem, ext = name, ""
    m = re.search(r"(\d+)$", stem)
    if m is None:
        return f"{stem}1{dot}{ext}"
    digits = m.group(1)
    width = len(digits)
    nxt = str(int(digits) + 1)
    if len(nxt) < width:
        nxt = nxt.zfill(width)
    return f"{stem[:m.start()]}{nxt}{dot}{ext}"

# Paper format presets (width_mm, height_mm). "No preset" leaves the fields
# untouched; selecting a preset pre-fills the width/height spin boxes.
_FORMAT_PRESETS = {
    "No preset": None,
    "A5 - landscape": (210, 148),
    "A5 - portrait": (148, 210),
    "A4 - landscape": (297, 210),
    "A4 - portrait": (210, 297),
    "A3": (297, 420),
    "Letter - portrait": (216, 279),
    "Letter - landscape": (279, 216),
    "Legal - portrait": (216, 356),
    "Tabloid - portrait": (279, 432),
}


def _format_for_size(width: int, height: int) -> str:
    """Return the preset name matching the given width/height, else "No preset".

    Iterates insertion order so exact-dimension matches win; the first preset
    whose (w, h) equals the requested size is selected. "No preset" is returned
    when nothing fits.
    """
    for name, dims in _FORMAT_PRESETS.items():
        if dims is not None and dims == (width, height):
            return name
    return "No preset"


class ScannerGUI(QMainWindow):
    def __init__(self, parent: Optional[QWidget] = None,
                 settings: Optional[QSettings] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Mustek ScanExpress A3 USB PRO")
        self.setMinimumSize(800, 600)
        self._scan_thread: Optional[ScanThread] = None
        self._preview_data: Optional[QImage] = None  # keep QImage alive
        self._settings = settings if settings is not None \
            else QSettings("mustek", "scanner")
        self._last_save_dir = self._settings.value(
            "save/dir", "", type=str)
        self._last_save_name = self._settings.value(
            "save/name", "mustek_scan.png", type=str)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # --- controls row 1: mode + dpi + trim top ---
        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Mode:"))
        self._combo_mode = QComboBox()
        self._combo_mode.addItems(_MODES)
        self._combo_mode.setCurrentText("color48")
        row1.addWidget(self._combo_mode)
        row1.addWidget(QLabel("DPI:"))
        self._combo_dpi = QComboBox()
        self._combo_dpi.addItems([str(d) for d in _DPIS])
        self._combo_dpi.setCurrentText("300")
        row1.addWidget(self._combo_dpi)
        row1.addWidget(QLabel("Trim top:"))
        self._spin_trim = QSpinBox()
        self._spin_trim.setRange(0, 50)
        self._spin_trim.setValue(4)
        self._spin_trim.setSuffix(" mm")
        row1.addWidget(self._spin_trim)
        row1.addStretch()
        layout.addLayout(row1)

        # --- controls row 2: format preset + width + height ---
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Format:"))
        self._combo_format = QComboBox()
        self._combo_format.addItems(_FORMAT_PRESETS.keys())
        self._combo_format.currentTextChanged.connect(self._on_format_changed)
        row2.addWidget(self._combo_format)
        row2.addWidget(QLabel("Width:"))
        self._spin_width = QSpinBox()
        self._spin_width.setRange(1, 300)
        self._spin_width.setValue(297)
        self._spin_width.setSuffix(" mm")
        row2.addWidget(self._spin_width)
        row2.addWidget(QLabel("Height:"))
        self._spin_height = QSpinBox()
        self._spin_height.setRange(1, 500)
        self._spin_height.setValue(210)
        self._spin_height.setSuffix(" mm")
        row2.addWidget(self._spin_height)
        row2.addStretch()
        layout.addLayout(row2)
        # Updating the format dropdown as the user edits width/height: changing
        # the combo triggers _on_format_changed which re-sets the spins (a
        # no-op when they already match), so no infinite loop results.
        self._spin_width.valueChanged.connect(self._on_size_changed)
        self._spin_height.valueChanged.connect(self._on_size_changed)
        # A4 landscape is the default preset: pre-fill the spin boxes to match
        # (the signal handler needs the spins to exist, so set it after they
        # are created above).
        self._combo_format.setCurrentText("A4 - landscape")
        self._restore_config()

        # --- preview area ---
        self._preview = QLabel("No scan yet")
        self._preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._preview.setMinimumHeight(200)
        # The label's size hint (the scaled pixmap) must not drive the scroll
        # area / window layout: otherwise each scan grows the preview area (the
        # pixmap scales to central.size(), which itself grew). Ignored policy
        # lets the scroll area control the label size; we scale the pixmap to
        # the viewport explicitly in _fit_preview.
        self._preview.setSizePolicy(QSizePolicy.Policy.Ignored,
                                    QSizePolicy.Policy.Ignored)
        self._scroll = QScrollArea()
        self._scroll.setWidget(self._preview)
        self._scroll.setWidgetResizable(True)
        layout.addWidget(self._scroll, stretch=1)

        # --- progress bar ---
        row4 = QHBoxLayout()
        self._progress = QProgressBar()
        self._progress.setVisible(False)
        row4.addWidget(self._progress, stretch=1)
        self._status_label = QLabel("Ready")
        row4.addWidget(self._status_label)
        layout.addLayout(row4)

        # --- buttons ---
        row5 = QHBoxLayout()
        self._btn_list = QPushButton("List Device")
        self._btn_list.clicked.connect(self._on_list)
        row5.addWidget(self._btn_list)
        row5.addStretch()
        self._btn_scan = QPushButton("Scan")
        self._btn_scan.clicked.connect(self._on_scan)
        row5.addWidget(self._btn_scan)
        self._btn_save = QPushButton("Save")
        self._btn_save.setEnabled(False)
        self._btn_save.clicked.connect(self._on_save)
        row5.addWidget(self._btn_save)
        self._btn_quick_save = QPushButton("Quick Save +1")
        self._btn_quick_save.setEnabled(False)
        self._btn_quick_save.clicked.connect(self._on_quick_save)
        row5.addWidget(self._btn_quick_save)
        layout.addLayout(row5)

    def _on_format_changed(self, text: str) -> None:
        """Pre-fill the width/height spin boxes from a format preset."""
        preset = _FORMAT_PRESETS.get(text)
        if preset is None:
            return
        width, height = preset
        self._spin_width.setValue(width)
        self._spin_height.setValue(height)

    def _on_size_changed(self) -> None:
        """Keep the format dropdown in sync with the width/height spin boxes:
        select the preset whose dimensions match, or "No preset" when none do."""
        self._combo_format.setCurrentText(
            _format_for_size(self._spin_width.value(),
                             self._spin_height.value()))

    def _restore_config(self) -> None:
        """Apply the persisted control values (mode/dpi/trim/format/width/height)."""
        mode = self._settings.value("config/mode", "color48", type=str)
        if mode in _MODES:
            self._combo_mode.setCurrentText(mode)
        dpi = self._settings.value("config/dpi", "300", type=str)
        if dpi in [str(d) for d in _DPIS]:
            self._combo_dpi.setCurrentText(dpi)
        trim = self._settings.value("config/trim_top", 4, type=int)
        self._spin_trim.setValue(trim)
        fmt = self._settings.value("config/format", "A4 - landscape", type=str)
        if fmt in _FORMAT_PRESETS:
            self._combo_format.setCurrentText(fmt)
        width = self._settings.value("config/width", 297, type=int)
        height = self._settings.value("config/height", 210, type=int)
        self._spin_width.setValue(width)
        self._spin_height.setValue(height)

    def _save_config(self) -> None:
        """Persist the current control values."""
        self._settings.setValue("config/mode", self._combo_mode.currentText())
        self._settings.setValue("config/dpi", self._combo_dpi.currentText())
        self._settings.setValue("config/trim_top", self._spin_trim.value())
        self._settings.setValue("config/format", self._combo_format.currentText())
        self._settings.setValue("config/width", self._spin_width.value())
        self._settings.setValue("config/height", self._spin_height.value())

    def closeEvent(self, event) -> None:
        self._save_config()
        super().closeEvent(event)

    def _on_scan(self) -> None:
        if self._scan_thread and self._scan_thread.isRunning():
            return
        self._btn_scan.setEnabled(False)
        self._btn_save.setEnabled(False)
        self._btn_quick_save.setEnabled(False)
        self._clear_preview()
        self._progress.setValue(0)
        self._progress.setVisible(True)
        self._status_label.setText("Starting scan...")
        dpi = int(self._combo_dpi.currentText())
        total = _scan_lines(self._spin_height.value(), dpi)
        self._progress.setMaximum(total)
        self._scan_thread = ScanThread(
            mode=self._combo_mode.currentText(),
            dpi=dpi,
            width=self._spin_width.value(),
            height=self._spin_height.value(),
            trim_top_mm=self._spin_trim.value(),
            scanner_factory=lambda: Scanner(LinuxUsbTransport()),
            parent=self,
        )
        self._scan_thread.progress.connect(self._on_progress)
        self._scan_thread.finished.connect(self._on_finished)
        self._scan_thread.error.connect(self._on_error)
        self._scan_thread.start()

    def _clear_preview(self) -> None:
        """Drop the previous scan's image so the preview area is empty on the
        next scan start."""
        self._preview_data = None
        self._preview.clear()
        self._preview.setText("No scan yet")

    def _on_progress(self, done: int, text: str) -> None:
        self._progress.setValue(done)
        self._status_label.setText(text)

    def _fit_preview(self) -> None:
        """Scale the stored preview image to the scroll-area viewport."""
        scroll = getattr(self, "_scroll", None)
        if scroll is None or self._preview_data is None:
            return
        size = scroll.viewport().size()
        self._preview.setPixmap(QPixmap.fromImage(self._preview_data).scaled(
            size,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        ))

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._fit_preview()

    def _on_finished(self, qimage) -> None:
        self._progress.setVisible(False)
        self._status_label.setText(
            f"Done: {qimage.width()}x{qimage.height()} px")
        self._btn_scan.setEnabled(True)
        self._btn_save.setEnabled(True)
        self._btn_quick_save.setEnabled(True)
        self._preview_data = qimage
        self._fit_preview()

    def _on_error(self, msg: str) -> None:
        self._progress.setVisible(False)
        self._status_label.setText(f"Error: {msg}")
        self._btn_scan.setEnabled(True)
        QMessageBox.warning(self, "Scan Error", msg)

    def _on_save(self) -> None:
        if not self._preview_data:
            return
        start_dir = self._last_save_dir or ""
        path, _ = QFileDialog.getSaveFileName(
            self, "Save scan as",
            os.path.join(start_dir, self._last_save_name),
            "Images (*.png *.jpg *.tiff)")
        if not path:
            return  # canceled
        self._save_to_path(path)

    def _on_quick_save(self) -> None:
        if not self._preview_data:
            return
        start_dir = self._last_save_dir or ""
        path = os.path.join(start_dir, _increment_name(self._last_save_name))
        self._save_to_path(path)

    def _save_to_path(self, path: str) -> None:
        fmt = "PNG"
        if path.lower().endswith((".jpg", ".jpeg")):
            fmt = "JPEG"
        elif path.lower().endswith((".tiff", ".tif")):
            fmt = "TIFF"
        if not self._preview_data.save(path, fmt):
            QMessageBox.warning(self, "Save Error", f"Failed to save to {path}")
            return
        self._status_label.setText(f"Saved: {path}")
        self._last_save_dir = os.path.dirname(path)
        self._last_save_name = os.path.basename(path)
        self._settings.setValue("save/dir", self._last_save_dir)
        self._settings.setValue("save/name", self._last_save_name)

    def _on_list(self) -> None:
        try:
            scanner = Scanner(LinuxUsbTransport())
            scanner.open()
            scanner.turn_power(1)
            scanner.determine_model_type()
            max_scan_dpi = max(scanner_reflective._DPI_TABLE_70)
            dev = scanner._tr.dev
            vid = dev.idVendor
            pid = dev.idProduct
            scanner.close()
            self._combo_dpi.clear()
            self._combo_dpi.addItems(
                [str(d) for d in _DPIS if d <= max_scan_dpi])
            if int(self._combo_dpi.currentText()) > max_scan_dpi or \
                    self._combo_dpi.currentText() == str(_DPIS[0]):
                self._combo_dpi.setCurrentText("300")
            if int(self._combo_dpi.currentText()) > max_scan_dpi:
                self._combo_dpi.setCurrentText(str(max_scan_dpi))
            QMessageBox.information(
                self, "Scanner Info",
                f"VID {vid:04X}  PID {pid:04X}\n"
                f"ScanExpress {max_scan_dpi} DPI")
        except Exception as e:
            QMessageBox.warning(self, "Scanner Not Found", str(e))


def run(argv: Optional[List[str]] = None) -> None:
    """Launch the scanning GUI. Entry point for python -m mustek_driver.gui."""
    app = QApplication(sys.argv if argv is None else argv)
    window = ScannerGUI()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    run()
