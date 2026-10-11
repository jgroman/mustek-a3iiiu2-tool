import logging
from typing import Any, Dict, List, Optional, Tuple

from mustek_driver import firmware, scanner_reflective
from mustek_driver.external import DriverError, trace

logger = logging.getLogger(__name__)

# ---- driver statics ----
g_scanner_opened = False
g_b_opened = False
g_attach_gate = 0
g_hardware_error = False
g_w_usb_speed = 0
g_scanner_initialized = False

# Full A3 platen in mm, the default scan window when none is given.
_DEFAULT_WINDOW_MM = (0, 0, 297, 420)

# The 1200-dpi defaults live in scanner_reflective._DPI_TABLE_60/70.
_DPI_TABLE_600 = [600, 300, 150, 75]

# Mode number -> scan-mode tag; legacy intermediate values 9/5/4 included.
_TAG_BY_NSCAN_MODE = {
    0: scanner_reflective.SCAN_MODE_COLOR48,   # public wScanMode 10
    4: scanner_reflective.SCAN_MODE_COLOR24,   # public wScanMode 2
    5: scanner_reflective.SCAN_MODE_GRAY16,    # public wScanMode 9
    9: scanner_reflective.SCAN_MODE_GRAY8,     # public wScanMode 1
    10: scanner_reflective.SCAN_MODE_TEXT8,    # public wScanMode 0
    scanner_reflective.SCAN_MODE_COLOR48: scanner_reflective.SCAN_MODE_COLOR48,
    scanner_reflective.SCAN_MODE_COLOR24: scanner_reflective.SCAN_MODE_COLOR24,
    scanner_reflective.SCAN_MODE_GRAY16: scanner_reflective.SCAN_MODE_GRAY16,
    scanner_reflective.SCAN_MODE_GRAY8: scanner_reflective.SCAN_MODE_GRAY8,
    scanner_reflective.SCAN_MODE_TEXT8: scanner_reflective.SCAN_MODE_TEXT8,
}

# bytes-per-pixel per tag, with SCAN_MODE_TEXT8 packed (>>3) instead.
_BYTES_PER_PIXEL_BY_TAG = {
    scanner_reflective.SCAN_MODE_COLOR48: 6,
    scanner_reflective.SCAN_MODE_COLOR24: 3,
    scanner_reflective.SCAN_MODE_GRAY16: 2,
    scanner_reflective.SCAN_MODE_GRAY8: 1,
}


class Scanner:
    def __init__(self, tr, services=None):
        self._tr = tr
        self._fw = firmware.Firmware(tr)
        self._reflective = scanner_reflective.ScannerReflective(tr)
        self._scanner_off: int = 0
        self._scan_params: Optional[Dict[str, Any]] = None
        self.dw_gain_g0: int = 0
        global g_scanner_opened
        g_scanner_opened = False

    @property
    def _scanner_opened(self) -> bool:
        return g_scanner_opened

    # ---- lifecycle ----
    def open(self) -> None:
        trace("< Open()")
        self._fw.open_connection(None)
        self._fw.init_chip()
        self._fw.back_home(1)
        self._fw.close_chip()
        self._fw.close_connection()
        trace("> Open()")

    def close(self) -> None:
        trace("< Close")
        global g_b_opened
        if g_b_opened:
            try:
                self._fw.open_connection(None)
                self._fw.init_chip()
                self._fw.back_home(1)
                self._fw.close_chip()
            except Exception as e:
                trace(">X Close (home skipped): %s", e)
            g_b_opened = False
        self._fw.close_connection()
        trace(">0 Close")

    def get_first_device(self) -> int:
        global g_scanner_opened
        if g_scanner_opened:
            return 2
        return 0

    def set_lamp(self, b_on: int):
        global g_attach_gate
        if g_attach_gate:
            raise DriverError("Prepare state>0")
        if b_on:
            raise DriverError("Unknown source")
        self._scanner_off = 1
        g_attach_gate = 1

    def turn_power(self, b_on: int) -> None:
        trace("< power_control")
        global g_hardware_error
        if g_hardware_error:
            trace("hardware error: turn_power")
            trace(">0 power_control")
            return
        self._fw.open_connection(None)
        self._fw.init_chip()
        self._fw.set_lamp(b_on)
        self._fw.close_chip()
        self._fw.close_connection()
        trace(">0 power_control")

    def carriage_home(self, b_wait: int) -> None:
        trace("< carriage_home")
        global g_hardware_error
        if g_hardware_error:
            trace("hardware error: back_home")
            trace(">0 carriage_home")
            return
        self._fw.open_connection(None)
        self._fw.init_chip()
        self._fw.back_home(b_wait)
        self._fw.close_chip()
        self._fw.close_connection()
        trace(">0 carriage_home")

    def release(self) -> None:
        trace("release lock in Release...")
        pass


    def start_scan(self, dpi: Optional[int] = None, mode: Optional[str] = None,
                   window: Optional[Tuple[int, int, int, int]] = None) -> None:
        trace("< start_scan")
        global g_b_opened
        try:
            if (self._scan_params is not None and dpi is None
                    and mode is None and window is None):
                params = self._scan_params
            else:
                if dpi is None:
                    raise DriverError(
                        "start_scan: dpi required when not using stored "
                        "scan parameters")
                params = self._mm_to_scan_params(
                    dpi, self._map_scan_mode(mode if mode is not None else "color"),
                    window if window is not None else _DEFAULT_WINDOW_MM)
                self._scan_params = params
            computed = self._scan_params_to_computed(params)
            logger.debug("start_scan params=%r", params)
            self._reflective.setup_scan(
                computed["n_scan_mode_tag"], computed["w_scan_dpi"],
                computed["w_line_dpi"], 0, computed["w_scan_start"],
                computed["w_speed"], computed["n_scan_width"],
                computed["w_reserved"])
            g_b_opened = True
        except Exception:
            trace("maybe_setup scan_pointer_error")
            raise
        trace(">O start_scan")

    def stop_scan(self) -> None:
        trace("< stop scan")
        try:
            self._reflective.scan_teardown()
        except Exception as e:
            trace(">X stop scan %s", e)
            raise
        trace(">0 stop scan")

    def read_scanned_data(self, count: int) -> bytes:
        trace("< read_scanned_data")
        trace("wWantedLineNum=%d", count)
        if count <= 0:
            return b""
        # The first pull_lines after prepare_scan discards one line (dw_flag42
        # "throw one line") and the 3-segment splice-init pass consumes one
        # line without output; pull extra so the ring holds `count` lines for
        # the getter.
        pull = count + 2 if self._reflective.dw_flag42 else count
        self._reflective.pull_lines(pull)
        total = [count]
        out = bytearray(count * self._reflective.dw_segment_bytes)
        self._reflective.get_row_data(out, total, 0, None)
        trace(">0 read_scanned_data")
        return bytes(out[:total[0] * self._reflective.dw_segment_bytes])

    def save_image(self, path, num_scan_lines: int, dpi: int) -> None:
        """Save scan lines directly to an image file.

        Reads `num_scan_lines` lines from the scanner via read_scanned_data(),
        then delegates to imageio.save_image() using the current scan mode
        and pixel width.

        Typical scan line counts (height_mm × dpi / 25.4):

        =========  ========  ========  ========
        Format     150 DPI   300 DPI   600 DPI
        =========  ========  ========  ========
        A4 (297mm)    1754      3508      7016
        A3 (420mm)    2480      4961      9921
        =========  ========  ========  ========

        Args:
            path: Output file path (.png, .jpg, .tiff).
            num_scan_lines: Number of scan lines (rows) to read and save.
            dpi: Scan resolution (stored in image metadata).
        """
        from mustek_driver import imageio
        if not g_scanner_opened:
            raise DriverError("Scanner not opened")
        lines = self.read_scanned_data(num_scan_lines)
        if not lines:
            raise DriverError("No scan data returned")
        # The processed output rows are dw_segment_bytes wide, not the raw CCD
        # pixel count (n_segment_pixels). Derive the image width from the row
        # size and the mode's bytes-per-pixel, or save_image misinterprets the
        # row boundaries (apparent per-row offset / black bands).
        bpp = imageio._bytes_per_pixel(self._reflective.dw_scan_mode)
        width = self._reflective.dw_segment_bytes // bpp
        return imageio.save_image(
            path, bytearray(lines),
            self._reflective.dw_scan_mode,
            width,
            dpi,
        )

    def set_parameters(self, params: Dict[str, Any]) -> None:
        w_scan_mode = params["w_scan_mode"]
        mode_map = {0: scanner_reflective.SCAN_MODE_TEXT8,
                    1: 9, 2: 4, 9: 5,
                    10: scanner_reflective.SCAN_MODE_COLOR48}
        trace("< set_parameters")
        if w_scan_mode not in mode_map:
            trace("scan_module_error")
            raise DriverError("scan module error")
        n_scan_mode = mode_map[w_scan_mode]
        if params.get("b_scan_source"):
            trace("scan_source_error")
            raise DriverError("scan source error")
        if params.get("b_pixel_flavor") not in (None, 0, 1):
            trace("pixel_flavor_error")
            raise DriverError("pixel flavor error")
        if w_scan_mode == 0 and params.get("w_threshold", 0) > 0x100:
            trace("wLinearThreshold_error")
            raise DriverError("wLinearThreshold error")
        x1, y1, x2, y2 = (params["w_x1"], params["w_y1"],
                          params["w_x2"], params["w_y2"])
        if x2 <= x1:
            trace("check_x1_x2_error")
            raise DriverError("check x1 x2 error")
        if y2 <= y1:
            trace("check_y1_y2_error")
            raise DriverError("check y1 y2 error")
        if x2 > 400:
            trace("MAX_PAPER_X_error,%d", x2)
            raise DriverError("MAX_PAPER_X error")
        if y2 > 580:
            trace("MAX_PAPER_Y_error,%d", y2)
            raise DriverError("MAX_PAPER_Y error")
        self._scan_params = self._mm_to_scan_params(params["w_dpi"],
                                                    n_scan_mode,
                                                    (x1, y1, x2, y2))
        logger.debug("set_parameters scan_params=%r", self._scan_params)
        trace("x1=%d", x1)
        global g_scanner_initialized
        if not g_scanner_initialized:
            self.set_lamp(0)
            g_scanner_initialized = True
        if "w_threshold" in params:
            self._reflective.b_threshold = params["w_threshold"] & 0xFF
        if "p_gamma_table" in params:
            self._reflective.p_gamma_table = params["p_gamma_table"]
        trace(">0 set_parameters")

    def get_scan_parameters(self) -> Dict[str, Any]:
        if self._scan_params is None:
            raise DriverError("get_scan_parameters: no scan parameters set "
                              "(call set_parameters() first)")
        trace("< get_parameters")
        computed = self._scan_params_to_computed(self._scan_params)
        trace("given sm_XDpi=%d", computed["w_scan_dpi"])
        trace("given sm_YDpi=%d", computed["w_line_dpi"])
        trace("given sm_BytesPerRow=%d", computed["n_bytes_per_line"])
        trace("given sm_Width=%d", computed["n_scan_width"])
        trace("given sm_Height=%d", computed["w_reserved"])
        trace(">0 get_parameters")
        return computed

    def get_device_info_string(self) -> str:
        global g_scanner_opened
        if g_scanner_opened:
            raise DriverError("scan in progress")
        return "UNKNOWN"

    def get_device_status(self) -> int:
        trace("< get_device_status")
        trace("Enter get_device_status")
        if self.get_first_device():
            trace(">0 get_device_status")
            return 0
        trace("> Leave the get_device_status with not-found")
        return 0x25

    def set_gain_g0(self, gain: int) -> None:
        self.dw_gain_g0 = gain

    def set_gain_g1(self, gain: int) -> None:
        pass

    def is_key_pressed(self) -> int:
        self._fw.open_connection(None)
        self._fw.open_chip()
        try:
            key = self._fw.check_function_key()
        finally:
            self._fw.close_connection()
        if key == 0x01:
            return 0x10
        if key == 0x02:
            return 0x02
        if key == 0x04:
            return 0x04
        if key == 0x08:
            return 0x08
        if key == 0x10:
            return 0x01
        return 0

    def determine_model_type(self) -> None:
        trace("decide if the scanner is 1200 or 600")
        if self.is_resolution_600():
            trace("this is ScanExpress 600!!!!!!!")
            trace("600DPI_Scanner")
            scanner_reflective._DPI_TABLE_70[:] = _DPI_TABLE_600
        else:
            trace("this is ScanExpress 1200!!!!!!!")
            trace("1200DPI_Scanner")

    def is_resolution_600(self) -> int:
        trace("<in IsResolution600")
        self._fw.open_connection(None)
        self._fw.init_chip()
        try:
            return self._fw.read_status_reg(2)
        finally:
            self._fw.close_chip()
            self._fw.close_connection()


    @staticmethod
    def _map_scan_mode(mode: str) -> int:
        if mode == "color":
            return scanner_reflective.SCAN_MODE_COLOR48
        if mode == "gray":
            return scanner_reflective.SCAN_MODE_GRAY16
        raise ValueError(f"unknown scan mode {mode!r}")

    @staticmethod
    def _mm_to_scan_params(dpi: int, n_scan_mode: int,
                           window: Tuple[int, int, int, int]) -> Dict[str, Any]:
        x1, y1, x2, y2 = window
        w_dpi = int(dpi)

        def px(v_mm):
            return int(v_mm * 10) * w_dpi // 254

        w_x1 = px(x1)
        w_y1 = px(y1)
        w_x2 = px(x2) - w_x1
        w_y2 = px(y2) - w_y1
        return {"n_scan_mode": n_scan_mode, "w_dpi": w_dpi,
                "w_x1": w_x1, "w_y1": w_y1, "w_x2": w_x2, "w_y2": w_y2}

    @classmethod
    def _scan_params_to_computed(cls, p: Dict[str, Any]) -> Dict[str, Any]:
        d = max(p["w_dpi"], 1)
        divisor = scanner_reflective.ScannerReflective._table_lookup(
            scanner_reflective._DPI_TABLE_70, d)
        line_dpi = scanner_reflective.ScannerReflective._table_lookup(
            scanner_reflective._DPI_TABLE_60, d)
        n_scan_mode_tag = _TAG_BY_NSCAN_MODE[p["n_scan_mode"]]
        n_scan_width = (p["w_x2"] * divisor) // d
        # TEXT8 is only reachable via set_parameters(w_scan_mode=0); start_scan
        # accepts only 'color'/'gray'.
        if n_scan_mode_tag == scanner_reflective.SCAN_MODE_TEXT8:
            n_bytes_per_line = n_scan_width >> 3
        else:
            n_bytes_per_line = n_scan_width * _BYTES_PER_PIXEL_BY_TAG.get(
                n_scan_mode_tag, 0)
        return {
            "n_scan_mode_tag": n_scan_mode_tag,
            "w_scan_dpi": divisor,
            "w_line_dpi": line_dpi,
            "w_scan_start": (p["w_x1"] * divisor) // d,
            "w_speed": (p["w_y1"] * line_dpi) // d,
            "n_scan_width": n_scan_width,
            "w_reserved": (p["w_y2"] * line_dpi) // d,
            "n_bytes_per_line": n_bytes_per_line,
        }
