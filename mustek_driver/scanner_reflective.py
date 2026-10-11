import logging
import random

from mustek_driver import config, firmware, node_pool
from mustek_driver.external import DriverError, trace

logger = logging.getLogger(__name__)

PIXEL_8BIT = 8
PIXEL_16BIT = 16
PIXEL_24BIT = 24
PIXEL_48BIT = 48

SCAN_MODE_COLOR48 = 0
SCAN_MODE_TEXT8 = 10
SCAN_MODE_COLOR24 = 15
SCAN_MODE_GRAY16 = 16
SCAN_MODE_GRAY8 = 20

# ---- module-level DPI divisor tables (set at runtime by model detection) ----
_DPI_TABLE_60 = [1200, 600, 300, 150, 75]
_DPI_TABLE_70 = [1200, 600, 300, 150, 75]

# used by shading_correct for the gamma interleave index.
_GAP_INTERLEAVE_LUT = bytes((
    0, 0, 0, 0, 1, 1, 2, 2, 4, 4, 5, 5, 8, 8, 9, 9,
    0, 0, 0, 0, 1, 1, 2, 2, 4, 4, 5, 5, 8, 8, 9, 9,
    0, 0, 0, 0, 1, 1, 2, 2, 4, 4, 5, 5, 8, 8, 9, 9,
    0, 0, 0, 0, 1, 1, 2, 2, 4, 4, 5, 5, 8, 8, 9, 9,
    1, 1, 1, 1, 3, 3, 3, 3, 6, 6, 6, 6, 10, 10, 11, 11,
    1, 1, 1, 1, 3, 3, 3, 3, 6, 6, 6, 6, 10, 10, 11, 11,
    2, 2, 2, 2, 3, 3, 3, 3, 7, 7, 7, 7, 10, 10, 11, 11,
    2, 2, 2, 2, 3, 3, 3, 3, 7, 7, 7, 7, 10, 10, 11, 11,
    4, 4, 4, 4, 6, 6, 7, 7, 12, 12, 12, 12, 13, 13, 14, 14,
    4, 4, 4, 4, 6, 6, 7, 7, 12, 12, 12, 12, 13, 13, 14, 14,
    5, 5, 5, 5, 6, 6, 7, 7, 12, 12, 12, 12, 13, 13, 14, 14,
    5, 5, 5, 5, 6, 6, 7, 7, 12, 12, 12, 12, 13, 13, 14, 14,
    8, 8, 8, 8, 10, 10, 10, 10, 13, 13, 13, 13, 15, 15, 15, 15,
    8, 8, 8, 8, 10, 10, 10, 10, 13, 13, 13, 13, 15, 15, 15, 15,
    9, 9, 9, 9, 11, 11, 11, 11, 14, 14, 14, 14, 15, 15, 15, 15,
    9, 9, 9, 9, 11, 11, 11, 11, 14, 14, 14, 14, 15, 15, 15, 15,
))


def _rand():
    """Return a pseudorandom integer in 0..0x7fff."""
    return random.getrandbits(15)


def _ftol(x):
    """Truncate a float toward zero."""
    return int(x)


class ScannerReflective:
    def __init__(self, tr):
        self.firmware = firmware.Firmware(tr)
        self.ring = node_pool.NodePool(0x2000, 0x20)
        self.w_scan_dpi = 0
        self.w_scan_dpi2 = 0
        self.w_line_dpi = 0
        self.dw_scan_mode = 0
        self.n_segments = 0
        self.n_segment_pixels = 0
        self.w_segment_count = 0
        self.b_threshold = 0x80
        self.b_pixel_format = PIXEL_8BIT
        self.dw_image_bytes = 0x7d000
        self.dw_lines_per_image = 0
        self.dw_buffer_size = 0x1800000
        self.dw_segment_bytes = 0
        self.w_speed = 0
        self.w_scan_start = 0
        self.n_scan_width = 0
        self.n_bytes_per_line = 0
        self.dw_line_bytes = 0
        self.dw_gap_blur = 0
        self.p_calib_r = None
        self.p_calib_g = None
        self.p_calib_b = None
        self.n_calib_offset = 0
        self.dw_step_count = 0
        self.n_sample = 0
        self.w_pixels = 0
        self.pfn_get_line = None
        self._gray8_16bit_raw = False  # set by setup_scan when GRAY8@1200 promotes raw to 16-bit
        self.dw_cancel_flag = 0
        self.w_flag54 = 0
        self.w_flag56 = 0
        self.dw_splice_alloc = 0
        self.dw_flag42 = 0
        self.p_big_buffer = None
        self.p_image_buffer = None
        self.p_line_buffer = None
        self.p_gamma_table = None  # set via set_gamma_table (default: no gamma)
        self.p_firmware_ring = None
        self.dw_splice_buf_a = None
        self.dw_splice_buf_b = None
        self.w_line_count = 0
        self.dw_ring_lines = 0
        self.dw_need_init = 0
        self.dw_total_lines = 0
        self.w_reserved = 0
        self._move_to_black_findleft = 0
        self._move_to_black_findtop = 0
        self._move_to_black_offset = 0
        self.firmware.dw_usb2_flag = 0
        self.load_settings()


    def lock(self):
        pass

    def unlock(self):
        pass

    # ---- line processors ----

    def _gray_requires_16bit_raw(self):
        return self.w_scan_dpi >= 0x259

    @staticmethod
    def _seg_consts(dpi):
        if dpi < 0x12D:
            if dpi == 300:
                return 300, 0x492, 0x26, 0xEE
            if dpi in (0x32, 0x4B, 0x96):
                return 0x96, 0x249, 0x11, 0x77
            return None
        if dpi == 600:
            return 600, 0x924, 0x50, 0x1DC
        if dpi == 0x4B0:
            return 0x4B0, 0x1248, 0x50, 0x1D8
        return None

    def process_gray_line_8bit(self, p_raw_line, dpi, x_start, width,
                               p_out_line, p_gamma_table):
        consts = self._seg_consts(dpi)
        if consts is None:
            return 1
        base, seg_pix, seg_offset, seg_margin = consts
        step = base // dpi
        u_start = step * x_start
        u_end = step * (x_start + width)
        seg2_off = seg_offset + seg_margin + seg_pix
        seg3_off = seg_offset + 2 * seg_margin + 2 * seg_pix
        out = bytearray(width)
        if p_gamma_table is None:
            for i, u in enumerate(range(u_start, u_end, step)):
                if u < seg_pix:
                    out[i] = p_raw_line[seg_offset + u]
                elif u < 2 * seg_pix:
                    out[i] = p_raw_line[seg2_off + (u - seg_pix)]
                elif u < 3 * seg_pix:
                    out[i] = p_raw_line[seg3_off + (u - 2 * seg_pix)]
        else:
            for i, u in enumerate(range(u_start, u_end, step)):
                if u < seg_pix:
                    b = p_raw_line[seg_offset + u]
                elif u < 2 * seg_pix:
                    b = p_raw_line[seg2_off + (u - seg_pix)]
                elif u < 3 * seg_pix:
                    b = p_raw_line[seg3_off + (u - 2 * seg_pix)]
                else:
                    continue
                idx = ((b << 4) | (_rand() & 0xF)) * 2
                out[i] = p_gamma_table[idx]
        p_out_line[:] = out
        if self.dw_gap_blur != 0:
            period = seg_pix // 15
            for i, u in enumerate(range(u_start + 1, u_end - 1, step)):
                if self.dw_gap_blur == 2 and not (_rand() < 0x4000):
                    continue
                if (u + 1) % period == 0:
                    out[i + 1] = _ftol(out[i + 1] * 0.67
                                       + out[min(i + 2, width - 1)] * 0.33) & 0xFF
                elif (u + 1) % period == 1:
                    out[i + 1] = _ftol(out[i + 1] * 0.67
                                       + out[i] * 0.33) & 0xFF
            p_out_line[:] = out
        return 0

    def process_gray_line_8bit_from16(self, p_raw_line, dpi, x_start, width,
                                      p_out_line, p_gamma_table):
        consts = self._seg_consts(dpi)
        if consts is None:
            return 1
        base, seg_pix, seg_offset, seg_margin = consts
        step = base // dpi
        u_start = step * x_start
        u_end = step * (x_start + width)
        raw = p_raw_line
        out = bytearray(width)
        for i, u in enumerate(range(u_start, u_end, step)):
            if u < seg_pix:
                off = (seg_offset + u) * 2
            elif u < 2 * seg_pix:
                off = (seg_offset + seg_margin + u) * 2
            else:
                off = (seg_offset + 2 * seg_margin + u) * 2
            v = raw[off] | (raw[off + 1] << 8)
            if p_gamma_table is not None:
                v = (p_gamma_table[v * 2] | (p_gamma_table[v * 2 + 1] << 8))
            out[i] = (v >> 8) & 0xFF
        p_out_line[:] = out
        if self.dw_gap_blur != 0:
            period = seg_pix // 15
            for i, u in enumerate(range(u_start + 1, u_end - 1, step)):
                if self.dw_gap_blur == 2 and not (_rand() < 0x4000):
                    continue
                if (u + 1) % period == 0:
                    out[i + 1] = _ftol(out[i + 1] * 0.67
                                       + out[min(i + 2, width - 1)] * 0.33) & 0xFF
                elif (u + 1) % period == 1:
                    out[i + 1] = _ftol(out[i + 1] * 0.67
                                       + out[i] * 0.33) & 0xFF
            p_out_line[:] = out
        return 0

    def process_color_line_16bit(self, p_raw_line, dpi, x_start, width,
                                 swap_rgb, p_out_line, p_gamma_table):
        consts = self._seg_consts(dpi)
        if consts is None:
            return 1
        base, seg_pix, seg_offset, seg_margin = consts
        step = base // dpi
        u_start = step * x_start
        u_end = step * (x_start + width)
        raw = p_raw_line
        out = bytearray(width * 6)
        for i, u in enumerate(range(u_start, u_end, step)):
            if u < seg_pix:
                off = (seg_offset + u) * 6
            elif u < 2 * seg_pix:
                off = (seg_offset + seg_margin + u) * 6
            else:
                off = (seg_offset + 2 * seg_margin + u) * 6
            r = raw[off] | (raw[off + 1] << 8)
            g = raw[off + 2] | (raw[off + 3] << 8)
            b = raw[off + 4] | (raw[off + 5] << 8)
            if p_gamma_table is not None:
                r = (p_gamma_table[r * 2] | (p_gamma_table[r * 2 + 1] << 8))
                g = (p_gamma_table[g * 2] | (p_gamma_table[g * 2 + 1] << 8))
                b = (p_gamma_table[b * 2] | (p_gamma_table[b * 2 + 1] << 8))
            o = i * 6
            if swap_rgb:
                out[o] = b & 0xFF
                out[o + 1] = (b >> 8) & 0xFF
                out[o + 4] = r & 0xFF
                out[o + 5] = (r >> 8) & 0xFF
            else:
                out[o] = r & 0xFF
                out[o + 1] = (r >> 8) & 0xFF
                out[o + 4] = b & 0xFF
                out[o + 5] = (b >> 8) & 0xFF
            out[o + 2] = g & 0xFF
            out[o + 3] = (g >> 8) & 0xFF
        p_out_line[:] = out
        if self.dw_gap_blur != 0:
            period = seg_pix // 15
            for i, u in enumerate(range(u_start + 1, u_end - 1, step)):
                if self.dw_gap_blur == 2 and not (_rand() < 0x4000):
                    continue
                o = (i + 1) * 6
                if (u + 1) % period == 0:
                    nxt = min((i + 2) * 6, len(out) - 6)
                elif (u + 1) % period == 1:
                    nxt = i * 6
                else:
                    continue
                for ch in range(3):
                    cur = out[o + ch * 2] | (out[o + ch * 2 + 1] << 8)
                    nb = out[nxt + ch * 2] | (out[nxt + ch * 2 + 1] << 8)
                    v = _ftol(cur * 0.67 + nb * 0.33)
                    out[o + ch * 2] = v & 0xFF
                    out[o + ch * 2 + 1] = (v >> 8) & 0xFF
            p_out_line[:] = out
        return 0

    def process_gray_line_16bit(self, p_raw_line, dpi, x_start, width,
                                p_out_line, p_gamma_table):
        consts = self._seg_consts(dpi)
        if consts is None:
            return 1
        base, seg_pix, seg_offset, seg_margin = consts
        step = base // dpi
        u_start = step * x_start
        u_end = step * (x_start + width)
        raw = p_raw_line
        out = bytearray(width * 2)
        for i, u in enumerate(range(u_start, u_end, step)):
            if u < seg_pix:
                off = (seg_offset + u) * 2
            elif u < 2 * seg_pix:
                off = (seg_offset + seg_margin + u) * 2
            else:
                off = (seg_offset + 2 * seg_margin + u) * 2
            v = raw[off] | (raw[off + 1] << 8)
            if p_gamma_table is not None:
                v = (p_gamma_table[v * 2] | (p_gamma_table[v * 2 + 1] << 8))
            out[i * 2] = v & 0xFF
            out[i * 2 + 1] = (v >> 8) & 0xFF
        p_out_line[:] = out
        if self.dw_gap_blur != 0:
            period = seg_pix // 15
            for i, u in enumerate(range(u_start + 1, u_end - 1, step)):
                if self.dw_gap_blur == 2 and not (_rand() < 0x4000):
                    continue
                o = (i + 1) * 2
                if (u + 1) % period == 0:
                    nxt = min((i + 2) * 2, len(out) - 2)
                elif (u + 1) % period == 1:
                    nxt = i * 2
                else:
                    continue
                cur = out[o] | (out[o + 1] << 8)
                nb = out[nxt] | (out[nxt + 1] << 8)
                v = _ftol(cur * 0.67 + nb * 0.33)
                out[o] = v & 0xFF
                out[o + 1] = (v >> 8) & 0xFF
            p_out_line[:] = out
        return 0

    def process_text_line_1bit(self, p_raw_line, dpi, x_start, width,
                               p_out_line):
        consts = self._seg_consts(dpi)
        if consts is None:
            return 1
        base, seg_pix, seg_offset, seg_margin = consts
        step = base // dpi
        u_start = step * x_start
        u_end = step * (x_start + width)
        seg2_off = seg_offset + seg_margin + seg_pix
        seg3_off = seg_offset + 2 * seg_margin + 2 * seg_pix
        out = bytearray(width >> 3)
        bit_counter = 0
        for u in range(u_start, u_end, step):
            if u < seg_pix:
                v = p_raw_line[seg_offset + u]
            elif u < 2 * seg_pix:
                v = p_raw_line[seg2_off + (u - seg_pix)]
            else:
                v = p_raw_line[seg3_off + (u - 2 * seg_pix)]
            if self.b_threshold < v:
                out[bit_counter >> 3] |= 0x80 >> (bit_counter & 0x1F)
            bit_counter += 1
            if bit_counter == 8:
                bit_counter = 0
        p_out_line[:] = out
        return 0

    def sort_and_average(self, vals, start, end):
        vals = list(vals)
        n = len(vals)
        for uv2 in range(n - 1):
            for uv3 in range(n - 1 - uv2):
                if vals[uv3] < vals[uv3 + 1]:
                    vals[uv3], vals[uv3 + 1] = (vals[uv3 + 1],
                                                      vals[uv3])
        count = end - start
        total = 0
        for uv2 in range(count):
            total += vals[start + uv2]
        return total // count

    # ---- ring pipeline (reader step + line getters) ----

    def store_line(self, line):
        self.ring.push_node_front(line)

    def read_line(self):
        return self.ring.pop_node()

    def pull_lines(self, dw_lines):
        trace("< read_data_from_scanner")
        self.firmware.n_state = firmware.FW_SCANNING
        raw = self.firmware.read_image(dw_lines)
        logger.debug("pull_lines: read %d lines (%d bytes)",
                     dw_lines, len(raw))
        for i in range(0, len(raw), self.dw_line_bytes):
            self.store_line(raw[i:i + self.dw_line_bytes])
            self.dw_total_lines += 1
        logger.debug("pull_lines: stored %d lines", dw_lines)
        if self.dw_flag42:
            # DMA batch, pop the first stored line and drop it. The ring is
            # FIFO, so the first pop is the first-stored line (raw[0:line_bytes]),
            self.dw_flag42 = 0
            trace("throw one line!")
            self.read_line()
        trace("here??")
        trace("> read_data_from_scanner, exit")
        return raw

    def get_row_data(self, p_out_line, pdw_total_lines, dw_line_idx, progress_cb):
        from mustek_driver import scanner as scanner_mod
        if not scanner_mod.g_scanner_opened:
            raise DriverError("get rows state<=0")
        if scanner_mod.g_attach_gate == 0:
            raise DriverError("get rows !state")
        if self.pfn_get_line is None:
            trace("maybe_get rows_pointer_error")
            raise DriverError("get row data: no line getter configured")
        logger.debug("get_row_data lines=%d", pdw_total_lines[0])
        return self.pfn_get_line(p_out_line, dw_line_idx, pdw_total_lines,
                                 progress_cb)

    def _splice_line(self, line):
        if self.w_segment_count != 3:
            return line
        if self.dw_splice_alloc:
            self.dw_splice_alloc = 0
            self.n_split_offset = self._get_split_offset(
                self.w_scan_dpi, self.b_pixel_format)
            self.dw_splice_size = self.dw_line_bytes - self.n_split_offset
            self.p_splice_buf_a = bytearray(self.dw_splice_size)
            self.p_splice_buf_b = bytearray(self.dw_splice_size)
            self.p_splice_buf_a[:] = line[self.n_split_offset:]
            return None
        self.p_splice_buf_b[:] = line[self.n_split_offset:]
        line[self.n_split_offset:] = self.p_splice_buf_a
        self.p_splice_buf_a[:] = self.p_splice_buf_b
        return line

    def _get_lines(self, flip, render, swap_rgb, p_out_line, p_total_lines,
                   progress_cb, stride):
        target = p_total_lines[0]
        done = 0
        self.dw_cancel_flag = 0
        out = memoryview(p_out_line)
        while done < target:
            if self.dw_cancel_flag != 0:
                trace("user cancel when scan or process...")
                raise DriverError("Hardware Error Cancel")
            line = self.read_line()
            if line is None:
                raise DriverError("Get*Line: scan-data ring underrun")
            line = bytearray(line)
            if flip is not None:
                flip(line, self.w_segment_count, self.w_scan_dpi)
            line = self._splice_line(line)
            if line is None:
                # reports progress on this read_line-success iteration with the
                # continuing.
                if progress_cb is not None and not progress_cb(done, target):
                    self.dw_cancel_flag = 1
                    raise DriverError("Cancelled")
                continue
            render(line, out[:stride], swap_rgb)
            done += 1
            self.w_line_count += 1
            out = out[stride:]
            if progress_cb is not None and not progress_cb(done, target):
                self.dw_cancel_flag = 1
                raise DriverError("Cancelled")
        # because it helps callers that pass a fresh list.
        p_total_lines[0] = done

    def get_color_line_16bit(self, p_out_line, swap_rgb, p_total_lines,
                             progress_cb):
        if p_out_line is None:
            raise DriverError("get_color_line_16bit: output buffer is None")
        self._get_lines(
            self._flip_line_data16,
            lambda line, out, sw: self.process_color_line_16bit(
                line, self.w_scan_dpi2, self.w_scan_start,
                self.n_segments, sw, out, self.p_gamma_table),
            swap_rgb, p_out_line, p_total_lines, progress_cb,
            self.dw_segment_bytes)

    def get_color_line_8bit(self, p_out_line, swap_rgb, p_total_lines,
                            progress_cb):
        if p_out_line is None:
            raise DriverError("get_color_line_8bit: output buffer is None")
        self._get_lines(
            self._flip_line_data,
            lambda line, out, sw: self._shading_correct(
                self.w_scan_dpi2, self.w_scan_start, self.n_segments,
                sw, line, out, self.p_gamma_table),
            swap_rgb, p_out_line, p_total_lines, progress_cb,
            self.dw_segment_bytes)

    def get_gray_line_16bit(self, p_out_line, swap_rgb, p_total_lines,
                            progress_cb):
        if p_out_line is None:
            raise DriverError("get_gray_line_16bit: output buffer is None")
        self._get_lines(
            None,
            lambda line, out, sw: self.process_gray_line_16bit(
                line, self.w_scan_dpi2, self.w_scan_start,
                self.n_segments, out, self.p_gamma_table),
            swap_rgb, p_out_line, p_total_lines, progress_cb,
            self.dw_segment_bytes)

    def get_gray_line_8bit(self, p_out_line, swap_rgb, p_total_lines,
                           progress_cb):
        if p_out_line is None:
            raise DriverError("get_gray_line_8bit: output buffer is None")
        if self._gray8_16bit_raw:
            self._get_lines(
                None,
                lambda line, out, sw: self.process_gray_line_8bit_from16(
                    line, self.w_scan_dpi2, self.w_scan_start,
                    self.n_segments, out, self.p_gamma_table),
                swap_rgb, p_out_line, p_total_lines, progress_cb,
                self.dw_segment_bytes)
        else:
            self._get_lines(
                None,
                lambda line, out, sw: self.process_gray_line_8bit(
                    line, self.w_scan_dpi2, self.w_scan_start,
                    self.n_segments, out, self.p_gamma_table),
                swap_rgb, p_out_line, p_total_lines, progress_cb,
                self.dw_segment_bytes)

    def get_text_line_1bit(self, p_out_line, swap_rgb, p_total_lines,
                           progress_cb):
        if p_out_line is None:
            raise DriverError("get_text_line_1bit: output buffer is None")
        self._get_lines(
            None,
            lambda line, out, sw: self.process_text_line_1bit(
                line, self.w_scan_dpi2, self.w_scan_start,
                self.n_segments, out),
            swap_rgb, p_out_line, p_total_lines, progress_cb,
            self.dw_segment_bytes >> 3)

    # ---- setup / teardown orchestration ----

    def setup_scan(self, dw_mode, w_scan_dpi, w_line_dpi, dw_reserved,
                   w_scan_start, w_speed, n_segments, w_reserved):
        # lazily import the scanner statics to avoid a circular import
        from mustek_driver import scanner as scanner_mod

        if scanner_mod.g_scanner_opened:
            raise DriverError("setup scan state>0")
        if scanner_mod.g_attach_gate == 0:
            raise DriverError("setup scan requires set_parameters() or "
                              "set_lamp() first (g_dwAttachGate==0)")

        trace("< setup scan(")
        self.dw_scan_mode = dw_mode
        self.w_scan_dpi2 = w_scan_dpi
        self.w_scan_dpi = w_scan_dpi
        self.w_line_dpi = w_line_dpi
        self.n_segments = n_segments
        self.w_reserved = w_reserved
        self.w_scan_start = w_scan_start
        self.w_speed = w_speed
        self.w_flag56 = 0
        self.w_flag54 = 0

        w_scan_width_var = self.w_scan_start + n_segments
        if self.w_line_dpi == 0x32:  # 50 dpi -> run at 150, 3x width
            self.w_scan_dpi = 0x96
            w_scan_width_var = w_scan_width_var * 3
        if self.w_line_dpi == 0x4B:  # 75 dpi -> run at 150, 2x width
            self.w_scan_dpi = 0x96
            w_scan_width_var = w_scan_width_var << 1

        w_dpi_sel = self.w_scan_dpi
        if w_dpi_sel < 0x12D:
            if w_dpi_sel == 300:
                if w_scan_width_var < 0x3CA:
                    self.w_segment_count = 1
                elif w_scan_width_var < 0x85C:
                    self.w_segment_count = 2
                else:
                    self.w_segment_count = 3
                self.n_segment_pixels = (
                    ((self.w_segment_count * 0x580 + -10) >> 4) << 4)
            elif w_dpi_sel in (0x32, 0x4B, 0x96):
                if w_scan_width_var < 0x181:
                    self.w_segment_count = 1
                elif w_scan_width_var < 0x3CA:
                    self.w_segment_count = 2
                else:
                    self.w_segment_count = 3
                self.n_segment_pixels = (
                    ((self.w_segment_count * 0x2C0 + -10) >> 4) << 4)
        elif w_dpi_sel == 600:
            if w_scan_width_var < 0x85C:
                self.w_segment_count = 1
            elif w_scan_width_var < 0x1180:
                self.w_segment_count = 2
            else:
                self.w_segment_count = 3
            self.n_segment_pixels = (
                ((self.w_segment_count * 0xB00 + -10) >> 4) << 4)
        elif w_dpi_sel == 0x4B0:
            if w_scan_width_var < 0x1180:
                self.w_segment_count = 1
            elif w_scan_width_var < 0x23C8:
                self.w_segment_count = 2
            else:
                self.w_segment_count = 3
            self.n_segment_pixels = (
                ((self.w_segment_count * 0x1420 + -10) >> 4) << 4)

        trace("Scan_Segments=%d, Scan_Width=%d",
              self.w_segment_count, w_scan_width_var)

        fw = self.firmware
        if self.dw_scan_mode == SCAN_MODE_COLOR48:
            self.dw_line_bytes = self.n_segment_pixels * 6
            fw.dw_line_bytes = self.dw_line_bytes
            self.dw_segment_bytes = self.n_segments * 6
            self.b_pixel_format = PIXEL_48BIT
        elif self.dw_scan_mode in (SCAN_MODE_TEXT8, SCAN_MODE_GRAY8):
            if (self.dw_scan_mode == SCAN_MODE_GRAY8
                    and self._gray_requires_16bit_raw()):
                # at the 1200 tier the ASIC outputs 16-bit samples even for an
                # (half the line), truncating every sample and producing dark
                # bands. Read the raw 16-bit data and down-convert to 8-bit in
                # the getter so GRAY8 stays an 8-bit scan. dw_line_bytes is the
                # raw line size (16-bit); dw_segment_bytes stays 8-bit output.
                self._gray8_16bit_raw = True
                self.dw_line_bytes = self.n_segment_pixels << 1
                fw.dw_line_bytes = self.dw_line_bytes
                self.dw_segment_bytes = self.n_segments
                self.b_pixel_format = PIXEL_16BIT
            else:
                self.dw_line_bytes = self.n_segment_pixels
                fw.dw_line_bytes = self.dw_line_bytes
                self.dw_segment_bytes = self.n_segments
                self.b_pixel_format = PIXEL_8BIT
        elif self.dw_scan_mode == SCAN_MODE_COLOR24:
            self.dw_line_bytes = self.n_segment_pixels * 3
            fw.dw_line_bytes = self.dw_line_bytes
            self.dw_segment_bytes = self.n_segments * 3
            self.b_pixel_format = PIXEL_24BIT
        elif self.dw_scan_mode == SCAN_MODE_GRAY16:
            self.dw_line_bytes = self.n_segment_pixels << 1
            fw.dw_line_bytes = self.dw_line_bytes
            self.dw_segment_bytes = self.n_segments << 1
            self.b_pixel_format = PIXEL_16BIT

        self.w_reserved = self.w_reserved + 2

        fw.open_connection(None)
        fw.init_chip()
        scanner_mod.g_scanner_opened = True
        if self.w_scan_dpi < 0x259:
            scanner_mod.g_w_usb_speed = 8000
        else:
            scanner_mod.g_w_usb_speed = 13000
        # global to the firmware's bUsbSpeedFlag field (the B0_CCD_DUMMY_CYCLES
        # register source, firmware.py); scanner.py declares no such global.
        fw.b_usb_speed_flag = 0
        # calibrations (load_file/adjust_ad/line_calibrate), which run set_scan_param
        # Without this the ctor default (0x40) makes the clamp range [195,255],
        # the stopped-motor LINE_TIME (227) passes unclamped, and the motor table
        # is built with period ~112 (no motor movement during the calibration).
        fw.w_usb_speed = scanner_mod.g_w_usb_speed

        fw.b_ad_trim_r = 0
        fw.b_ad_trim_g = 0
        fw.b_ad_trim_b = 0
        fw.a_ad_offset_r = [0x19, 0x19, 0x19]
        fw.a_ad_offset_g = [0x14, 0x14, 0x14]
        fw.w_lamp_gain_r = 100
        fw.w_lamp_gain_g = 100
        fw.w_lamp_gain_b = 100
        fw.w_lamp_offset_r = 0x709
        fw.w_lamp_offset_g = 0x709
        fw.w_lamp_offset_b = 0x709

        self._load_file()
        self._adjust_ad()

        # calibrate runs at line_dpi = w_scan_dpi and needs the same monotonic
        # w_usb_speed as the main scan; the initial 8000/13000 would give a
        # different LINE_TIME clamp and different calibration gains).
        if fw.dw_usb2_flag == 1:
            scanner_mod.g_w_usb_speed = self._usb_speed_usb2()
        else:
            scanner_mod.g_w_usb_speed = self._usb_speed_usb1()
        fw.w_usb_speed = scanner_mod.g_w_usb_speed

        self.w_scan_start = _ftol(self.w_scan_start
                                  + self.w_scan_dpi
                                  * self._move_to_black_findleft / 600.0)

        if self.b_pixel_format < PIXEL_24BIT:
            self._line_calibrate_gray()
        else:
            self._line_calibrate_color()
        self._check_usb_version()

        trace("user_left=%d, findleft=%d, m_X=%d",
              self.w_scan_start, self._move_to_black_findleft,
              self._move_to_black_offset)
        trace("user_top=%d, findtop=%d",
              self.w_speed, self._move_to_black_findtop)

        self.w_speed = _ftol(self._move_to_black_findtop
                             + self.w_speed * 600.0 / self.w_line_dpi)

        dw_speed_target = self._speed_target(fw.dw_usb2_flag)
        if dw_speed_target + 2 < self.w_speed:
            trace("move_forword=%d", self.w_speed - dw_speed_target)
            if self.w_speed - dw_speed_target < 0x2FE:
                fw.calculate_scan_param(0, self.w_speed - dw_speed_target, 1)
            else:
                if fw.dw_usb2_flag == 0:
                    self.w_speed = self.w_speed + self._speed_adjust()
                else:
                    self.w_speed = self.w_speed + self._speed_adjust_usb2()
                fw.calculate_scan_param(0, self.w_speed - dw_speed_target, 1)
            self.w_speed = 0
        elif self.w_speed < dw_speed_target - 2:
            trace("move_back=%d", dw_speed_target - self.w_speed)
            fw.calculate_scan_param(1, dw_speed_target - self.w_speed, 1)
            self.w_speed = 0

        if fw.dw_usb2_flag == 1:
            trace("USB2.0")
            scanner_mod.g_w_usb_speed = self._usb_speed_usb2()
        else:
            trace("USB1.1")
            scanner_mod.g_w_usb_speed = self._usb_speed_usb1()
        # the firmware field or CCD_PIX and the motor-time computation use
        # the ctor default (0x40) instead. (Already set before the line_calibrate
        # above; re-propagate here so the value is current for the main scan.)
        fw.w_usb_speed = scanner_mod.g_w_usb_speed

        trace("LAST_PIXEL_number_is:%d", self.n_segment_pixels)
        fw.set_scan_param(self.b_pixel_format, self.w_scan_dpi,
                          self.w_line_dpi, 0, self.w_speed,
                          self.n_segment_pixels, self.w_reserved,
                          1, 1, 0, 0)
        # table, which overwrites the speed table built during the line
        # calibration. Rebuild the speed table here (before prepare_scan) so the
        # scan reads with a valid shading speed table — otherwise the scan data
        # is ~60% zeros (use_speed_table=1 sparse) vs dense when rebuilt.
        # TODO(workaround): the calibration speed table (which would carry a
        # per-column green boost at 1200) is NOT restored here: re-uploading it
        # before prepare_scan makes the scan read sparse/dark on this hardware
        # (the 1200 shading addressing is unsupported here), so the working path
        # scans with the flat buffer and no per-column speed-table gain. Any
        # 1200-dpi green boost must therefore be applied in software at the
        # output stage (process_color_line_16bit / _shading_correct), not in the
        # speed-table gain.
        if False and getattr(self, "p_speed_table_gain", None) is not None:
            fw.build_speed_table(self.p_speed_table_gain,
                                 self.p_speed_table_avg,
                                 self.w_speed_table_dpi,
                                 self.w_speed_table_count)
        self.prepare_scan()
        trace(">0 setup scan")

    def _speed_target(self, dw_usb2_flag):
        u = self.w_line_dpi
        if dw_usb2_flag == 0:
            if u < 0x12D:
                if u == 300:
                    return 0x18 if self.b_pixel_format >= PIXEL_24BIT else 0x50
                if u == 0x4B:
                    return 0x70
                if u == 0x96:
                    return 0x70
            elif u == 600:
                return 0x43
            elif u == 0x4B0:
                return 0x10
        else:
            if u < 0x12D:
                if u == 300:
                    return 0x54
                if u == 0x4B:
                    return 0x72
                if u == 0x96:
                    return 0x72
            elif u == 600:
                return 0x41
            elif u == 0x4B0:
                return 0x0E
        return 0

    def _speed_adjust(self):
        u = self.w_line_dpi
        if u < 0x12D:
            if u == 300:
                return 0x17 if self.b_pixel_format >= PIXEL_24BIT else 0x14
            if u == 0x4B:
                return 0x23
            if u == 0x96:
                return 0x17
        elif u == 600:
            return 0x19
        elif u == 0x4B0:
            return 0x12
        return 0

    def _speed_adjust_usb2(self):
        u = self.w_line_dpi
        if u < 0x12D:
            if u == 300:
                return 0x17
            if u == 0x4B:
                return 0x17
            if u == 0x96:
                return 0x15
        elif u == 600:
            return 0x17
        elif u == 0x4B0:
            return 0x17
        return 0

    def _usb_speed_usb1(self):
        u = self.w_line_dpi
        if u < 0x12D:
            if u == 300:
                return 3000 if self.b_pixel_format < PIXEL_24BIT else 0x1B46
            if u == 0x4B:
                return 0x1B5D
            if u == 0x96:
                return 3000 if self.b_pixel_format < PIXEL_24BIT else 5000
        elif u == 600:
            if self.b_pixel_format < PIXEL_24BIT:
                return 6000
            if self.b_pixel_format < PIXEL_48BIT:
                return 28000
            return 40000
        elif u == 0x4B0:
            if self.b_pixel_format < PIXEL_24BIT:
                return 18000
            if self.b_pixel_format < PIXEL_48BIT:
                return 28000
            return 40000
        return 0

    def _usb_speed_usb2(self):
        u = self.w_line_dpi
        if u < 0x12D:
            if u == 300:
                return 0x1B46
            if u == 0x4B:
                return 0x1B5D
            if u == 0x96:
                return 0xF3C
        elif u == 600:
            if self.b_pixel_format < PIXEL_48BIT:
                return 6000
            return 0x30D4
        elif u == 0x4B0:
            if self.b_pixel_format < PIXEL_24BIT:
                return 9000
            if self.b_pixel_format < PIXEL_48BIT:
                return 0x30D4
            return 25000
        return 0

    def _check_usb_version(self):
        reply = self.firmware.t.read_command(0xD0, bytes(5), sub=0)
        if (reply[0] & 1) == 1:
            self.firmware.dw_usb2_flag = 1
            trace("USB Host = USB2.0")
        else:
            self.firmware.dw_usb2_flag = 0
            trace("USB Host = USB1.1")

    # ---- calibration helpers (load_file/move_to_black, adjust_ad, line_calibrate)

    def _load_file(self):
        trace("< move_to_black")
        fw = self.firmware
        buf1 = bytearray(0x39210)
        fw.set_scan_param(PIXEL_8BIT, 600, 600, 0, 0, 0xAF0, 100, 1, 0, 0, 0)
        fw.scan_start()
        p_dest = bytearray(fw.read_data_dma(280000))
        fw.usb_command_write()
        fw.n_state = firmware.FW_SCAN_READY
        trace("before draw gray data")
        out_line = bytearray(0x924)
        for i in range(100):
            self.process_gray_line_8bit(
                p_dest[i * 0xAF0:(i + 1) * 0xAF0], 600, 0, 0x924,
                out_line, None)
            buf1[i * 0x924:(i + 1) * 0x924] = out_line
        trace("end draw gray data")
        # 100 lines (tmp_1c = 0x64), recording the found line offset.
        tmp_4c = 0x32  # default scan offset
        tmp_2c = 0
        tmp_28 = 0
        tmp_1c = 0x64
        while (tmp_1c & 0xFFFF) > tmp_28:
            tmp_30 = 100
            while 10 < tmp_30:
                base = tmp_28 * 0x924
                tmp_14 = (buf1[base + tmp_30]
                            + buf1[base + tmp_30 - 1]
                            + buf1[base + tmp_30 - 2]
                            + buf1[base + tmp_30 - 3]
                            + buf1[base + tmp_30 - 4]) // 5
                tmp_18 = (buf1[base + tmp_30 - 5]
                            + buf1[base + tmp_30 - 6]
                            + buf1[base + tmp_30 - 7]
                            + buf1[base + tmp_30 - 8]
                            + buf1[base + tmp_30 - 9]) // 5
                if (tmp_2c == 0 and tmp_18 < tmp_14
                        and 100 < tmp_14 - tmp_18):
                    tmp_4c = tmp_28
                    tmp_2c = 1
                tmp_30 -= 1
            if tmp_2c != 0:
                break
            tmp_28 += 1
        trace("Move And Find Result:TOPSIDE=%d,", tmp_4c)
        iv1 = ((tmp_4c & 0xFFFF) - (tmp_1c & 0xFFFF)) + 100
        tmp_24 = ((iv1 + ((iv1 >> 0x1F) & 3)) >> 2) << 2
        if (tmp_24 & 0xFFFF) < 3:
            tmp_24 = 3
        trace("move_forword:%d", tmp_24)
        fw.calculate_scan_param(0, tmp_24 & 0xFFFF, 1)
        trace("move_topleft_end")
        # find-result offsets: left=0x0C, top=0x6A
        self._move_to_black_findleft = 0x0C
        self._move_to_black_findtop = 0x6A
        self._move_to_black_offset = tmp_4c
        trace("find offset=%d", tmp_4c + 0xFE)
        trace(">0 move_to_black")

    def _adjust_ad(self):
        fw = self.firmware
        fw.b_ad_trim_r = 0
        fw.b_ad_trim_g = 0
        fw.b_ad_trim_b = 0
        fw.a_ad_offset_r = [0, 0, 0]
        fw.a_ad_offset_g = [0x80, 0x80, 0x80]
        fw.w_lamp_gain_r = 100
        fw.w_lamp_gain_g = 100
        fw.w_lamp_gain_b = 100
        fw.w_lamp_offset_r = 0x65
        fw.w_lamp_offset_g = 0x65
        fw.w_lamp_offset_b = 0x65
        fw.b_usb_speed_flag = 0
        if self.w_scan_dpi < 0x259:
            dw_lo_count = 0x20F0
            dw_hi_count = 0x1B6C
            w_dpi = 600
        else:
            dw_lo_count = 0x3C50
            dw_hi_count = 0x36D8
            w_dpi = 0x4B0
        p_calib_raw = bytearray(dw_lo_count * 3)
        p_calib_corrected = bytearray(dw_hi_count * 3)
        self._calculate_max_min(self.w_scan_dpi, dw_hi_count)
        fw.set_scan_param(PIXEL_24BIT, w_dpi, w_dpi, 0, 0, dw_lo_count,
                          1, 0, 0, 1, 0)
        w_target_max, w_target_min = 6, 5
        b_trim_rhi = b_trim_ghi = b_trim_bhi = 0
        b_trim_rlo = b_trim_glo = b_trim_blo = 0xFF
        n_loop = 0
        while True:
            trace("AD:Offset(%d,%d,%d), Gain(%d,%d,%d)",
                  fw.a_ad_offset_g[0], fw.a_ad_offset_g[1], fw.a_ad_offset_g[2],
                  fw.w_lamp_gain_r, fw.w_lamp_gain_g, fw.w_lamp_gain_b)
            fw.set_scan_param(PIXEL_24BIT, w_dpi, w_dpi, 0, 0, dw_lo_count,
                              1, 0, 0, 1, 1)
            fw.scan_start()
            p_calib_raw = bytearray(fw.read_data_dma(dw_lo_count * 3))
            fw.usb_command_write()
            fw.n_state = firmware.FW_SCAN_READY  # re-arm for next scan_start
            self._flip_line_data(p_calib_raw, 3, w_dpi)
            self._shading_correct(w_dpi, 0, dw_hi_count, 0, p_calib_raw,
                                  p_calib_corrected, None)
            w_min_r, w_min_g, w_min_b = self._calc_min_rgb(p_calib_corrected)
            trace("AD:R_Min=%d, G_Min=%d, B_Min=%d", w_min_r, w_min_g, w_min_b)
            if fw.b_ad_trim_r == 0:
                if w_target_max < w_min_r:
                    if fw.a_ad_offset_g[0] < 3:
                        fw.b_ad_trim_r = 1
                        b_trim_rhi, b_trim_rlo = 0, 0xFF
                        fw.a_ad_offset_g[0] = 0x80
                    else:
                        fw.a_ad_offset_g[0] = (
                            ((b_trim_rlo + b_trim_rhi) // 2 + b_trim_rhi) // 2)
                        b_trim_rlo = (b_trim_rlo + b_trim_rhi) // 2
                elif w_min_r < w_target_min:
                    fw.a_ad_offset_g[0] = (
                        ((b_trim_rlo + b_trim_rhi) // 2 + b_trim_rlo) // 2)
                    b_trim_rhi = (b_trim_rlo + b_trim_rhi) // 2
            elif w_target_max < w_min_r:
                fw.a_ad_offset_g[0] = (
                    ((b_trim_rlo + b_trim_rhi) // 2 + b_trim_rlo) // 2)
                b_trim_rhi = (b_trim_rlo + b_trim_rhi) // 2
            elif w_min_r < w_target_min:
                if fw.a_ad_offset_g[0] < 3:
                    fw.b_ad_trim_r = 0
                    b_trim_rhi, b_trim_rlo = 0, 0xFF
                    fw.a_ad_offset_g[0] = 0x80
                else:
                    fw.a_ad_offset_g[0] = (
                        ((b_trim_rhi + b_trim_rlo) // 2 + b_trim_rhi) // 2)
                    b_trim_rlo = (b_trim_rhi + b_trim_rlo) // 2
            if fw.b_ad_trim_g == 0:
                if w_target_max < w_min_g:
                    if fw.a_ad_offset_g[1] < 3:
                        fw.b_ad_trim_g = 1
                        b_trim_ghi, b_trim_glo = 0, 0xFF
                        fw.a_ad_offset_g[1] = 0x80
                    else:
                        fw.a_ad_offset_g[1] = (
                            ((b_trim_ghi + b_trim_glo) // 2 + b_trim_ghi) // 2)
                        b_trim_glo = (b_trim_ghi + b_trim_glo) // 2
                elif w_min_g < w_target_min:
                    fw.a_ad_offset_g[1] = (
                        ((b_trim_glo + b_trim_ghi) // 2 + b_trim_glo) // 2)
                    b_trim_ghi = (b_trim_ghi + b_trim_glo) // 2
            elif w_target_max < w_min_g:
                fw.a_ad_offset_g[1] = (
                    ((b_trim_glo + b_trim_ghi) // 2 + b_trim_glo) // 2)
                b_trim_ghi = (b_trim_ghi + b_trim_glo) // 2
            elif w_min_g < w_target_min:
                if fw.a_ad_offset_g[1] < 3:
                    fw.b_ad_trim_g = 0
                    b_trim_ghi, b_trim_glo = 0, 0xFF
                    fw.a_ad_offset_g[1] = 0x80
                else:
                    fw.a_ad_offset_g[1] = (
                        ((b_trim_glo + b_trim_ghi) // 2 + b_trim_ghi) // 2)
                    b_trim_glo = (b_trim_glo + b_trim_ghi) // 2
            if fw.b_ad_trim_b == 0:
                if w_target_max < w_min_b:
                    if fw.a_ad_offset_g[2] < 3:
                        fw.b_ad_trim_b = 1
                        b_trim_bhi, b_trim_blo = 0, 0xFF
                        fw.a_ad_offset_g[2] = 0x80
                    else:
                        fw.a_ad_offset_g[2] = (
                            ((b_trim_blo + b_trim_bhi) // 2 + b_trim_bhi) // 2)
                        b_trim_blo = (b_trim_bhi + b_trim_blo) // 2
                elif w_min_b < w_target_min:
                    fw.a_ad_offset_g[2] = (
                        ((b_trim_blo + b_trim_bhi) // 2 + b_trim_blo) // 2)
                    b_trim_bhi = (b_trim_bhi + b_trim_blo) // 2
            elif w_target_max < w_min_b:
                fw.a_ad_offset_g[2] = (
                    ((b_trim_blo + b_trim_bhi) // 2 + b_trim_blo) // 2)
                b_trim_bhi = (b_trim_bhi + b_trim_blo) // 2
            elif w_min_b < w_target_min:
                if fw.a_ad_offset_g[2] < 3:
                    fw.b_ad_trim_b = 0
                    b_trim_bhi, b_trim_blo = 0, 0xFF
                    fw.a_ad_offset_g[2] = 0x80
                else:
                    fw.a_ad_offset_g[2] = (
                        ((b_trim_blo + b_trim_bhi) // 2 + b_trim_bhi) // 2)
                    b_trim_blo = (b_trim_blo + b_trim_bhi) // 2
            if (w_target_min <= w_min_r <= w_target_max
                    and w_target_min <= w_min_g <= w_target_max
                    and w_target_min <= w_min_b <= w_target_max):
                trace("good_anlog!!!!, offset_good!!!!")
                break
            n_loop += 1
            logger.debug("adjust_ad offset loop iter=%d trim=(%d,%d,%d)",
                         n_loop, fw.b_ad_trim_r, fw.b_ad_trim_g, fw.b_ad_trim_b)
            if n_loop >= 0x1E:
                break
        dw_lamp_hi_r, dw_lamp_hi_g, dw_lamp_hi_b = 0, 0x578, 0x578
        dw_lamp_lo_r, dw_lamp_lo_g, dw_lamp_lo_b = 0x578, 0, 0
        fw.w_lamp_offset_r = 1000
        fw.w_lamp_offset_g = 1000
        fw.w_lamp_offset_b = 1000
        w_gain_target_hi = 0xAA
        w_gain_target_lo = 0xA0
        n_loop = 0
        # BEFORE the lamp-gain loop. This full setup issues the ACTION=0x03
        # the carriage from the black bar to the bright lamp reference. The
        # lamp-gain reads stayed at the black bar and the exposure converged
        fw.set_scan_param(PIXEL_24BIT, w_dpi, w_dpi, 0, 0, dw_lo_count,
                          1, 0, 0, 1, 0)
        while True:
            trace("AD:Lamp(%d-%d,%d-%d,%d-%d), Offset(...)",
                  dw_lamp_hi_r, dw_lamp_lo_r, dw_lamp_hi_g, dw_lamp_lo_g,
                  dw_lamp_hi_b, dw_lamp_lo_b)
            b_ok = [False, False, False]
            fw.set_scan_param(PIXEL_24BIT, w_dpi, w_dpi, 0, 0, dw_lo_count,
                              1, 0, 0, 1, 1)
            fw.scan_start()
            p_calib_raw = bytearray(fw.read_data_dma(dw_lo_count * 3))
            fw.usb_command_write()
            fw.n_state = firmware.FW_SCAN_READY  # re-arm for next scan_start
            self._flip_line_data(p_calib_raw, 3, w_dpi)
            self._shading_correct(w_dpi, 0, dw_hi_count, 0, p_calib_raw,
                                  p_calib_corrected, None)
            w_max_r, w_max_g, w_max_b = self._calc_max_rgb(p_calib_corrected)
            if w_gain_target_hi < w_max_r:
                fw.w_lamp_offset_r = (
                    (dw_lamp_hi_r + dw_lamp_hi_g) // 2 + dw_lamp_hi_r) // 2 + 300
                dw_lamp_hi_g = (dw_lamp_hi_r + dw_lamp_hi_g) // 2
            elif w_max_r < w_gain_target_lo:
                fw.w_lamp_offset_r = (
                    (dw_lamp_hi_r + dw_lamp_hi_g) // 2 + dw_lamp_hi_g) // 2 + 300
                dw_lamp_hi_r = (dw_lamp_hi_r + dw_lamp_hi_g) // 2
            else:
                b_ok[0] = True
            if w_gain_target_hi < w_max_g:
                fw.w_lamp_offset_g = (
                    (dw_lamp_lo_g + dw_lamp_hi_b) // 2 + dw_lamp_lo_g) // 2 + 300
                dw_lamp_hi_b = (dw_lamp_lo_g + dw_lamp_hi_b) // 2
            elif w_max_g < w_gain_target_lo:
                fw.w_lamp_offset_g = (
                    (dw_lamp_lo_g + dw_lamp_hi_b) // 2 + dw_lamp_hi_b) // 2 + 300
                dw_lamp_lo_g = (dw_lamp_lo_g + dw_lamp_hi_b) // 2
            else:
                b_ok[1] = True
            if w_gain_target_hi < w_max_b:
                fw.w_lamp_offset_b = (
                    (dw_lamp_lo_b + dw_lamp_lo_r) // 2 + dw_lamp_lo_b) // 2 + 300
                dw_lamp_lo_r = (dw_lamp_lo_b + dw_lamp_lo_r) // 2
            elif w_max_b < w_gain_target_lo:
                fw.w_lamp_offset_b = (
                    (dw_lamp_lo_b + dw_lamp_lo_r) // 2 + dw_lamp_lo_r) // 2 + 300
                dw_lamp_lo_b = (dw_lamp_lo_b + dw_lamp_lo_r) // 2
            else:
                b_ok[2] = True
            if all(b_ok):
                trace("lamp_time, good_anlog!!")
                break
            n_loop += 1
            logger.debug("adjust_ad lamp loop iter=%d", n_loop)
            if 0x0E < n_loop:
                break
        trace("AD:Last_Lamp(%d-%4d,%4d,%4d)",
              dw_lamp_hi_r, dw_lamp_lo_r, dw_lamp_hi_g, dw_lamp_lo_g)
        self.free_calib_buffers()

    def _line_calibrate_gray(self):
        trace("< line_calibrate_gray")
        fw = self.firmware
        uv1 = self.n_segment_pixels
        w_scan_dpi = self.w_scan_dpi
        fw.w_lamp_gain_r = 100
        fw.w_lamp_offset_r = 0x65
        fw.w_lamp_gain_b = 100
        fw.w_lamp_offset_b = 0x65
        trace("Gray: Gain(%d,%d,%d), Offset(%d,%d,%d)",
              fw.w_lamp_gain_r, fw.w_lamp_gain_g, fw.w_lamp_gain_b,
              fw.w_lamp_offset_r, fw.w_lamp_offset_g, fw.w_lamp_offset_b)
        fw.set_scan_param(PIXEL_16BIT, w_scan_dpi, 600, 0, 0, uv1,
                          0x28, 1, 0, 1, 0)
        fw.scan_start()
        p_dest = bytearray(fw.read_data_dma(uv1 * 0x50))
        fw.usb_command_write()
        fw.n_state = firmware.FW_SCAN_READY
        fw.set_scan_param(PIXEL_16BIT, w_scan_dpi, 600, 0, 0, uv1,
                          0x28, 0, 0, 1, 0)
        fw.set_lamp(0)
        fw.scan_start()
        p_dest_00 = bytearray(fw.read_data_dma(uv1 * 0x50))
        fw.usb_command_write()
        fw.n_state = firmware.FW_SCAN_READY

        self._line_calibrate_build(uv1, w_scan_dpi, gray=True,
                                   p_lamp=p_dest, p_dark=p_dest_00)
        trace(">0 line_calibrate_gray")

    def _line_calibrate_color(self):
        trace("< line_calibrate_color")
        fw = self.firmware
        uv1 = self.n_segment_pixels
        w_scan_dpi = self.w_scan_dpi
        fw.set_scan_param(PIXEL_48BIT, w_scan_dpi, 600, 0, 0, uv1,
                          0x40, 1, 0, 1, 0)
        fw.scan_start()
        p_dest = bytearray(fw.read_data_dma(uv1 * 0x180))
        fw.usb_command_write()
        fw.n_state = firmware.FW_SCAN_READY
        fw.set_scan_param(PIXEL_48BIT, w_scan_dpi, w_scan_dpi, 0, 0, uv1,
                          0x40, 0, 0, 1, 0)
        fw.set_lamp(0)
        fw.scan_start()
        p_dest_00 = bytearray(fw.read_data_dma(uv1 * 0x180))
        fw.usb_command_write()
        fw.n_state = firmware.FW_SCAN_READY
        self._line_calibrate_build(uv1, w_scan_dpi, gray=False,
                                   p_lamp=p_dest, p_dark=p_dest_00)
        trace(">0 line_calibrate_color")

    def _line_calibrate_build(self, w_entry_count, w_scan_dpi, gray, p_lamp,
                              p_dark):
        fw = self.firmware
        n_count = 0x28 if gray else 0x40  # sample window (gray 0x28 / color 0x40)
        bpp = 2 if gray else 6            # bytes per sample
        channels = 1 if gray else 3
        p_avg = bytearray(w_entry_count * 6)
        p_gain = bytearray(w_entry_count * 6)
        # table from the dark read and the gain diff as
        # previously sampled p_avg from p_lamp and diff = dark - lamp (inverted
        # buffers), producing bright avgs and clamped/noisy gains. Verified
        # (1803/3911/1811), gain_R = 0x24EC (9452) from lamp-dark = 13.
        for i in range(w_entry_count):
            for ch in range(channels):
                acc = []
                for k in range(n_count):
                    off = (k * w_entry_count + i) * bpp + ch * 2
                    acc.append(p_dark[off] | (p_dark[off + 1] << 8))
                avg = self.sort_and_average(acc, 0x12, 0x1A)
                p_avg[i * 6 + ch * 2] = avg & 0xFF
                p_avg[i * 6 + ch * 2 + 1] = (avg >> 8) & 0xFF
        if gray:
            # gray funclet zeroes the G/B slots of the average table
            for i in range(w_entry_count):
                p_avg[i * 6 + 2] = p_avg[i * 6 + 3] = 0
                p_avg[i * 6 + 4] = p_avg[i * 6 + 5] = 0
        for i in range(w_entry_count):
            for ch in range(channels):
                acc = []
                for k in range(n_count):
                    off = (k * w_entry_count + i) * bpp + ch * 2
                    acc.append(p_lamp[off] | (p_lamp[off + 1] << 8))
                avg = self.sort_and_average(acc, 0x12, 0x1A)
                base_avg = (p_avg[i * 6 + ch * 2]
                            | (p_avg[i * 6 + ch * 2 + 1] << 8))
                diff = (avg & 0xFFFF) - (base_avg & 0xFFFF)
                if diff <= 0:
                    v = 0x2000
                else:
                    v = _ftol(65535.0 / diff * 8192.0)
                p_gain[i * 6 + ch * 2] = v & 0xFF
                p_gain[i * 6 + ch * 2 + 1] = (v >> 8) & 0xFF
        if gray:
            # gray funclet pins the G/B gain slots to 0x2000
            for i in range(w_entry_count):
                p_gain[i * 6 + 2] = p_gain[i * 6 + 4] = 0x00
                p_gain[i * 6 + 3] = p_gain[i * 6 + 5] = 0x20
        fw.build_speed_table(p_gain, p_avg, w_scan_dpi, w_entry_count)
        # Keep the calibration gain/avg tables so the speed table can be rebuilt
        # right before the main scan (set_scan_param's motor-table upload
        # invalidates the speed table built here).
        self.p_speed_table_gain = p_gain
        self.p_speed_table_avg = p_avg
        self.w_speed_table_dpi = w_scan_dpi
        self.w_speed_table_count = w_entry_count
        fw.calculate_scan_param(1, 0x30, 1)

    # ---- small calibration helpers ----

    def _flip_line_data(self, p_line, n_segments, w_scan_dpi):
        if w_scan_dpi < 0x259:
            if w_scan_dpi == 600:
                r0, r1, r2, r3, r4 = 0xAD0, 0xB00, 0x15D0, 0x1600, 0x20D0
            elif w_scan_dpi == 0x96:
                r0, r1, r2, r3, r4 = 0x290, 0x2C0, 0x550, 0x580, 0x810
            elif w_scan_dpi == 300:
                r0, r1, r2, r3, r4 = 0x550, 0x580, 0xAD0, 0xB00, 0x1050
            else:
                return
        elif w_scan_dpi == 0x4B0:
            r0, r1, r2, r3, r4 = 0x13F0, 0x1420, 0x2810, 0x2840, 0x3C30
        else:
            return
        if n_segments not in (1, 2, 3):
            return
        if n_segments == 3:
            for i in range((r4 - r3) // 2):
                a, b = (i + r3) * 3 + 1, ((r4 - i) - 1) * 3 + 1
                p_line[a], p_line[b] = p_line[b], p_line[a]
        if n_segments != 1:
            for i in range((r2 - r1) // 2):
                a, b = (i + r1) * 3 + 1, ((r2 - i) - 1) * 3 + 1
                p_line[a], p_line[b] = p_line[b], p_line[a]
        for i in range(r0 // 2):
            a, b = i * 3 + 1, ((r0 - i) - 1) * 3 + 1
            p_line[a], p_line[b] = p_line[b], p_line[a]

    def _flip_line_data16(self, p_line, n_segments, w_scan_dpi):
        if w_scan_dpi < 0x259:
            if w_scan_dpi == 600:
                r0, r1, r2, r3, r4 = 0xAD0, 0xB00, 0x15D0, 0x1600, 0x20D0
            elif w_scan_dpi == 0x96:
                r0, r1, r2, r3, r4 = 0x290, 0x2C0, 0x550, 0x580, 0x810
            elif w_scan_dpi == 300:
                r0, r1, r2, r3, r4 = 0x550, 0x580, 0xAD0, 0xB00, 0x1050
            else:
                return
        elif w_scan_dpi == 0x4B0:
            r0, r1, r2, r3, r4 = 0x13F0, 0x1420, 0x2810, 0x2840, 0x3C30
        else:
            return
        if n_segments not in (1, 2, 3):
            return
        if n_segments == 3:
            for i in range((r4 - r3) // 2):
                a, b = (i + r3) * 6 + 2, ((r4 - i) - 1) * 6 + 2
                p_line[a:a + 2], p_line[b:b + 2] = p_line[b:b + 2], p_line[a:a + 2]
        if n_segments != 1:
            for i in range((r2 - r1) // 2):
                a, b = (i + r1) * 6 + 2, ((r2 - i) - 1) * 6 + 2
                p_line[a:a + 2], p_line[b:b + 2] = p_line[b:b + 2], p_line[a:a + 2]
        for i in range(r0 // 2):
            a, b = i * 6 + 2, ((r0 - i) - 1) * 6 + 2
            p_line[a:a + 2], p_line[b:b + 2] = p_line[b:b + 2], p_line[a:a + 2]

    @staticmethod
    def _get_split_offset(w_scan_dpi, b_pixel_format):
        if w_scan_dpi < 0x259:
            if w_scan_dpi == 600:
                off = 0x1600
            elif w_scan_dpi == 0x96:
                off = 0x580
            elif w_scan_dpi == 300:
                off = 0xB00
            else:
                off = 0
        elif w_scan_dpi == 0x4B0:
            off = 0x2840
        else:
            off = 0
        if b_pixel_format == PIXEL_16BIT:
            return off << 1
        if b_pixel_format == PIXEL_24BIT:
            return off * 3
        if b_pixel_format == PIXEL_48BIT:
            return off * 6
        if b_pixel_format == PIXEL_8BIT:
            return off
        return 0

    def _shading_correct(self, dpi, x_start, width, swap_rgb, p_raw_line,
                         p_out_line, p_gamma_table):
        if dpi < 0x12D:
            if dpi == 300:
                base, seg_pix, seg_offset, seg_margin = 300, 0x492, 0x26, 0xEE
            elif dpi in (0x32, 0x4B, 0x96):
                base, seg_pix, seg_offset, seg_margin = 0x96, 0x249, 0x11, 0x77
            else:
                return 1
        elif dpi == 600:
            base, seg_pix, seg_offset, seg_margin = 600, 0x924, 0x50, 0x1DC
        elif dpi == 0x4B0:
            base, seg_pix, seg_offset, seg_margin = 0x4B0, 0x1249, 0x50, 0x1D7
        else:
            return 1
        step = base // dpi
        u_start = step * x_start
        u_end = step * (x_start + width)
        out = bytearray(width * 3)
        for i, u in enumerate(range(u_start, u_end, step)):
            if u < seg_pix:
                off = (seg_offset + u) * 3
            elif u < 2 * seg_pix:
                off = (seg_offset + seg_margin + u) * 3
            else:
                off = (seg_offset + 2 * seg_margin + u) * 3
            r, g, b = p_raw_line[off], p_raw_line[off + 1], p_raw_line[off + 2]
            o = i * 3
            if p_gamma_table is None:
                if swap_rgb:
                    out[o], out[o + 1], out[o + 2] = b, g, r
                else:
                    out[o], out[o + 1], out[o + 2] = r, g, b
            else:
                lut = _GAP_INTERLEAVE_LUT
                out[o] = p_gamma_table[
                    ((r << 4) | lut[(g & 0xF) + (b & 0xF) * 0x10]) * 2]
                out[o + 1] = p_gamma_table[
                    ((g << 4) | lut[(b & 0xF) + (r & 0xF) * 0x10]) * 2 + 0x2000]
                out[o + 2] = p_gamma_table[
                    ((b << 4) | lut[(r & 0xF) + (g & 0xF) * 0x10]) * 2 + 0x4000]
        p_out_line[:] = out
        if self.dw_gap_blur != 0:
            period = seg_pix // 15
            for i, u in enumerate(range(u_start + 1, u_end - 3, step)):
                if self.dw_gap_blur == 2 and not (_rand() < 0x4000):
                    continue
                extra = 0
                if dpi == 0x4B0:
                    # 1200-dpi segment-boundary pointer bump (shading_correct
                    # pixel 2 before a boundary, shifts the window.
                    if ((u_start > seg_pix and u == u_start + 1)
                            or (u == seg_pix - 2)):
                        extra += 3
                    if u_start > 2 * seg_pix and u == u_start + 1:
                        extra += 6
                    elif u == 2 * seg_pix - 4:
                        extra += 3
                o = (i + 1) * 3 + extra
                if (u + 1) % period == 0:
                    nxt = min(o + 3, len(out) - 3)
                elif (u + 1) % period == 1:
                    nxt = o - 3
                else:
                    continue
                for ch in range(3):
                    v = _ftol(out[o + ch] * 0.67 + out[nxt + ch] * 0.33)
                    out[o + ch] = v & 0xFF
            p_out_line[:] = out
        return 0

    def _calc_min_rgb(self, p_line_data):
        iv1 = self.n_calib_offset
        n_buckets = self.n_sample // 3
        step = self.dw_step_count
        mins = [0xFFFF] * 3
        for n_pix in range(n_buckets):
            acc = [0, 0, 0]
            for n_ch in range(step):
                for ch in range(3):
                    off = ((n_pix * step + n_ch
                            + (step * self.n_sample // 3) * ch) * 3 + iv1)
                    acc[ch] += (p_line_data[off] + p_line_data[off + 1]
                                + p_line_data[off + 2])
            for ch in range(3):
                v = acc[ch] // (step * 3)
                if v < mins[ch]:
                    mins[ch] = v
        return mins[0], mins[1], mins[2]

    def _calc_max_rgb(self, p_line_data):
        iv1 = self.n_calib_offset
        step = self.dw_step_count
        maxs = [0] * 3
        for n_pix in range(self.n_sample):
            acc = [0, 0, 0]
            for n_ch in range(step):
                off = (n_pix * step + n_ch) * 3 + iv1
                for ch in range(3):
                    acc[ch] += p_line_data[off + ch]
            for ch in range(3):
                v = acc[ch] // step
                if v > maxs[ch]:
                    maxs[ch] = v
        return maxs[0], maxs[1], maxs[2]

    def _calculate_max_min(self, dpi, w_pixels):
        self.n_calib_offset = 0
        self.w_pixels = w_pixels
        if dpi < 0x259:
            # (199 < dpi) ? 0 : 0xffffffff, & 0xffffffec, +0x1e, truncated uint
            v = (0 if 199 < dpi else 0xFFFFFFFF) & 0xFFFFFFEC
            self.dw_step_count = (v + 0x1E) & 0xFFFFFFFF
        else:
            self.dw_step_count = 0x32
        self.w_pixels = self.w_pixels - self.n_calib_offset
        self.n_sample = self.w_pixels // self.dw_step_count
        self.p_calib_r = bytearray(self.n_sample * 2)
        self.p_calib_g = bytearray(self.n_sample * 2)
        self.p_calib_b = bytearray(self.n_sample * 2)

    def free_calib_buffers(self):
        for attr in ("p_calib_r", "p_calib_g", "p_calib_b"):
            setattr(self, attr, None)

    # ---- prepare_scan / scan_teardown / get_scan_parameters / load_settings

    def prepare_scan(self):
        self.dw_lines_per_image = self.dw_image_bytes // self.dw_line_bytes
        self.dw_ring_lines = self.dw_buffer_size // self.dw_line_bytes
        if self.dw_lines_per_image:
            self.dw_ring_lines = ((self.dw_ring_lines // self.dw_lines_per_image)
                                  * self.dw_lines_per_image)
        self.dw_need_init = 1
        self.dw_total_lines = 0
        self.w_line_count = 0
        # line getter by mode. COLOR24 -> get_color_line_8bit (not the gray
        # processor that earlier tasks used as a placeholder).
        if self.dw_scan_mode == SCAN_MODE_COLOR48:
            self.pfn_get_line = self.get_color_line_16bit
        elif self.dw_scan_mode == SCAN_MODE_TEXT8:
            self.pfn_get_line = self.get_text_line_1bit
        elif self.dw_scan_mode == SCAN_MODE_COLOR24:
            self.pfn_get_line = self.get_color_line_8bit
        elif self.dw_scan_mode == SCAN_MODE_GRAY16:
            self.pfn_get_line = self.get_gray_line_16bit
        elif self.dw_scan_mode == SCAN_MODE_GRAY8:
            self.pfn_get_line = self.get_gray_line_8bit
        self.p_big_buffer = bytearray(self.dw_buffer_size)
        self.p_image_buffer = bytearray(self.dw_image_bytes)
        self.p_line_buffer = bytearray(self.dw_line_bytes)
        self.dw_splice_alloc = 1
        self.dw_flag42 = 1
        self.ring.drain()
        self.firmware.scan_start()

    def scan_teardown(self):
        from mustek_driver import scanner as scanner_mod
        if not scanner_mod.g_scanner_opened:
            trace("no_wait!")
            raise DriverError("stop scan !state")
        if scanner_mod.g_attach_gate != 0:
            self.lock()
            self.dw_cancel_flag = 1
            self.unlock()
            trace("need_wait!")
            self.p_big_buffer = None
            self.p_image_buffer = None
            self.p_line_buffer = None
            self.p_firmware_ring = None
            self.dw_splice_buf_a = None
            self.dw_splice_buf_b = None
            trace("safe_now...")
            self.firmware.usb_command_write()
            self.firmware.close_connection()
            scanner_mod.g_scanner_opened = False
        else:
            raise DriverError("stop scan !state")

    def get_scan_parameters(self):
        divisor = self._table_lookup(_DPI_TABLE_70, self.w_scan_start)
        line_dpi = self._table_lookup(_DPI_TABLE_60, self.w_scan_start)
        n_scan_start = (self.w_speed * divisor) // max(self.w_scan_start, 1)
        w_speed = (self.n_scan_width * line_dpi) // max(self.w_scan_start, 1)
        n_scan_width = (self.w_reserved * divisor) // max(self.w_scan_start, 1)
        w_reserved = (self.dw_line_bytes * line_dpi) // max(self.w_scan_start, 1)
        mode = self.dw_scan_mode
        if mode == SCAN_MODE_TEXT8:
            n_scan_width = ((n_scan_width + 7) >> 3) << 3
            if n_scan_width < 8:
                n_scan_width = 8
        if mode == SCAN_MODE_COLOR48:
            n_scan_mode_tag = 0
            n_bytes_per_line = n_scan_width * 6
        elif mode == SCAN_MODE_COLOR24:
            n_scan_mode_tag = 0xF
            n_bytes_per_line = n_scan_width * 3
        elif mode == SCAN_MODE_GRAY16:
            n_scan_mode_tag = 0x10
            n_bytes_per_line = n_scan_width << 1
        elif mode == SCAN_MODE_GRAY8:
            n_scan_mode_tag = 0x14
            n_bytes_per_line = n_scan_width
        elif mode == SCAN_MODE_TEXT8:
            n_scan_mode_tag = 10
            n_bytes_per_line = n_scan_width >> 3
        else:
            n_scan_mode_tag = 0
            n_bytes_per_line = 0
        return {
            "n_scan_mode_tag": n_scan_mode_tag,
            "w_scan_dpi": self.w_scan_dpi,
            "w_line_dpi": self.w_line_dpi,
            "n_scan_start": n_scan_start,
            "w_speed": w_speed,
            "n_scan_width": n_scan_width,
            "n_bytes_per_line": n_bytes_per_line,
        }

    @staticmethod
    def _table_lookup(table, value):
        """Largest table entry <= value; last entry if none qualifies."""
        last = 0
        for entry in table:
            if entry == 0:
                break
            if entry <= value:
                return entry
            last = entry
        return last if last else table[0]

    def load_settings(self, path=None):
        trace("load_settings path=%s", path or "<defaults>")
        settings = dict(config.DEFAULTS["setting"])
        if path:
            merged = config.load_ini(path)
            settings.update(merged["setting"])
        self.dw_gap_blur = int(settings.get("Gap_Blur", 1))
        return settings
