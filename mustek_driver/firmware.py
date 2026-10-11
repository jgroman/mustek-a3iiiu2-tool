import logging
import struct
import time

from mustek_driver import motor_tables, registers, transport
from mustek_driver.external import DriverError, DriverTimeoutError, trace

logger = logging.getLogger(__name__)

FW_IDLE = 0
FW_CONNECTED = 1
FW_SCAN_READY = 2
FW_SCANNING = 3

OPEN_MAGIC = (b"\x64\x64\x64\x64", b"\x65\x65\x65\x65",
              b"\x44\x44\x44\x44", b"\x45\x45\x45\x45")
CLOSE_MAGIC = (b"\x64\x64\x64\x64", b"\x65\x65\x65\x65",
               b"\x16\x16\x16\x16", b"\x17\x17\x17\x17")


class TableDesc:

    def __init__(self, b_op=0, b_bank_mode=0, w_addr_lo=0, w_addr_hi_flags=0,
                 dw_size=0, b_flag=0, p_data=b""):
        self.b_op = b_op
        self.b_bank_mode = b_bank_mode
        self.w_addr_lo = w_addr_lo
        self.w_addr_hi_flags = w_addr_hi_flags
        self.dw_size = dw_size
        self.b_flag = b_flag
        self.p_data = p_data


class Firmware:

    def __init__(self, tr):
        self.t = tr
        self.n_state = FW_IDLE
        self._bank = -1
        self.dw_line_bytes = 0
        self.dw_init_timing1 = 60000
        self.a_ad_offset_r = [0x28, 0x28, 0x28]
        self.a_ad_offset_g = [0x78, 0x28, 0x32]
        self.a_ad_offset_b = [0x78, 0x28, 0x32]
        self.b_ad_trim_r = 0
        self.b_ad_trim_g = 1
        self.b_ad_trim_b = 1
        self.dw_motor_speed = 0x5DC
        self.dw_motor_table_addr = 0
        self.dw_motor_table_dma_addr = 0
        self._table_dma_chunk = 0x8000
        self.dw_scan_line_timing_mode = 0
        self.w_motor_speed1 = 200
        self.w_motor_speed2 = 0x2EE
        self.w_motor_speed3 = 200
        self.w_motor_speed4 = 2000
        self.w_motor_speed5 = 200
        self.w_motor_speed6 = 0x640
        self.b_lamp_current0 = 0x64
        self.b_lamp_current1 = 0x64
        self.n_dram_test_flag = 0
        self.w_back_speed = 2631
        self.w_usb_speed = 0x40
        self.b_chip_open = 0
        self._buffered_first = b""
        self._buffered_full = False
        self.b_usb_speed_flag = 0
        self.dw_speed_table_addr = 0
        self.dw_afe_adcclk_timing = 0xF0F0F000
        self.dw_afe_adcvs_timing = 0x03FC0000
        self.dw_afe_adcrs_timing = 0x03FC0000
        self.w_afe_chalatch_pos = 0x0C08
        self.w_afe_chblatch_pos = 0x0208
        self.w_afe_chclatch_pos = 0x0C10
        self.w_afe_chdlatch_pos = 0x1400
        self.b_afe_secondary_fflatch_pos = 10
        self.dw_ccd_dummy_cycle_timing = 0xFFFF0000  # -0x10000 as uint
        self.dw_ccd_ph1_timing = 0x0FFF0000
        self.b_phtg_pulse_width = 2
        self.b_phtg_wait_width = 1
        self.b_phtg_timing_adj = 0
        self.b_phtg_timing_setup = 6
        self.b_lamp_current2 = 0x5A
        self.b_lamp_current3 = 0x5A
        self.b_lamp_current150_a = 0x50
        self.b_lamp_current150_b = 0x50
        self.b_lamp_current300_a = 0x46
        self.b_lamp_current300_b = 0x46
        self.b_lamp_current600_a = 0x2D
        self.b_lamp_current600_b = 0x2D
        self.b_lamp_current1200_a = 0x28
        self.b_lamp_current1200_b = 0x28
        self.b_lamp_current2400_a = 0x3C
        self.b_lamp_current2400_b = 0x3C
        # lamp gain/offset (written by write_analog_ctrls)
        self.w_lamp_gain_r = 0
        self.w_lamp_offset_r = 0
        self.w_lamp_gain_g = 0
        self.w_lamp_offset_g = 0
        self.w_lamp_gain_b = 0
        self.w_lamp_offset_b = 0

    # ---- register I/O ----

    def set_reg_immediate(self, reg: int, value: int) -> None:
        bank = 0 if reg < 0x100 else (1 if reg < 0x200 else 2)
        if reg < 0x100:
            if self._bank != 0:
                self.t.send_command(0xB0, bytes((0x5F, 0, 0x5F, 0)))
                self._bank = 0
        elif reg < 0x200:
            if self._bank != 1:
                self.t.send_command(0xB0, bytes((0x5F, 1, 0x5F, 1)))
                self._bank = 1
        else:
            if reg < 0x300 and self._bank != 2:
                self.t.send_command(0xB0, bytes((0x5F, 2, 0x5F, 2)))
                self._bank = 2
        lo = reg & 0xFF
        logger.debug("set_reg 0x%02X=0x%02X bank=%d", reg, value & 0xFF, bank)
        self.t.send_command(0xB0, bytes((lo, value, lo, value)))

    def set_reg_buffered(self, reg: int, value: int) -> None:
        if reg < 0x100:
            if self._bank != 0:
                self.t.send_command(0xB0, bytes((0x5F, 0, 0x5F, 0)))
                self._bank = 0
        elif reg < 0x200:
            if self._bank != 1:
                self.t.send_command(0xB0, bytes((0x5F, 1, 0x5F, 1)))
                self._bank = 1
        else:
            if reg < 0x300 and self._bank != 2:
                self.t.send_command(0xB0, bytes((0x5F, 2, 0x5F, 2)))
                self._bank = 2
        pair = bytes((reg & 0xFF, value))
        if self._buffered_full:
            self.t.send_command(0xB0, self._buffered_first + pair)
            self._buffered_full = False
        else:
            self._buffered_first = pair
            self._buffered_full = True

    def read_data_dma(self, size: int) -> bytes:
        self.clear_fifos()
        out = bytearray()
        off = 0
        while off < size:
            n = min(transport.DMA_CHUNK, size - off)
            logger.debug("dma_read chunk offset=0x%X size=0x%X", off, n)
            self.set_scan_buffer_address(1, n)
            self.t.send_command(0x03, bytes((n & 0xFF, (n >> 8) & 0xFF, 0, 0)))
            out += self.t.bulk_read(n)
            off += n
        return bytes(out)

    def write_data_dma(self, size: int, data: bytes) -> None:
        self.clear_fifos()
        off = 0
        while off < size:
            n = min(transport.DMA_CHUNK, size - off)
            logger.debug("dma_write chunk offset=0x%X size=0x%X", off, n)
            self.set_scan_buffer_address(0, n)
            self.t.send_command(0x02, bytes((n & 0xFF, (n >> 8) & 0xFF, 0, 0)))
            self.t.bulk_write(data[off:off + n])
            off += n
            # chunk (both full 0x8000 chunks and the tail).
            time.sleep(0.02)

    def write_data_bulk(self, data: bytes) -> None:
        self.t.bulk_write(data)

    def read_data_bulk(self, size: int) -> bytes:
        return self.t.bulk_read(size)

    def set_scan_buffer_address(self, b_halfword: int, addr: int) -> None:
        b = (addr >> 0x18) & 0xFF
        if b_halfword == 0:
            vals = ((addr & 0xFF), ((addr >> 8) & 0xFF),
                    ((addr >> 16) & 0xFF), b)
        else:
            vals = (((addr >> 1) & 0xFF), ((addr >> 9) & 0xFF),
                    ((addr >> 17) & 0xFF), (b >> 1))
        for i, v in enumerate(vals):
            self.set_reg_immediate(0x7C + i, v)

    def set_dma_address(self, addr: int, bank: int) -> None:
        w_payload_len = ((addr >> 0x10) & 0xFFFF) << 16 | (bank & 0xFFFF)
        self.t.send_command(4, bytes((bank, bank, bank, bank)),
                            sub=4, payload_len=w_payload_len)

    def set_motor_table_addr(self, lo: int, hi: int, extend: int) -> None:
        self.set_reg_immediate(registers.B0_HOST_ADDR_S_L, lo & 0xFF)
        self.set_reg_immediate(registers.B0_HOST_ADDR_S_H, (lo >> 8) & 0xFF)
        if extend == 0:
            self.set_reg_immediate(registers.B0_HOST_ADDR_S_E, (lo >> 16) & 0xFF)
        else:
            self.set_reg_immediate(registers.B0_HOST_ADDR_S_E,
                                   ((lo >> 16) & 0xFF) | 0x80)
        self.set_reg_immediate(registers.B0_HOST_ADDR_E_L, hi & 0xFF)
        self.set_reg_immediate(registers.B0_HOST_ADDR_E_H, (hi >> 8) & 0xFF)
        self.set_reg_immediate(registers.B0_HOST_ADDR_E_E, (hi >> 16) & 0xFF)
        self.clear_fifos()

    # ---- motor tables ----

    def t_cmd_chunk(self) -> int:
        return self._table_dma_chunk

    def write_table_block(self, desc) -> None:
        lo = desc.w_addr_lo
        self.set_reg_immediate(registers.B0_HOST_ADDR_S_L, lo & 0xFF)
        hi_flags = desc.w_addr_hi_flags
        if desc.b_bank_mode == 0x02:
            self.set_reg_immediate(registers.B0_HOST_ADDR_S_H, (lo >> 8) & 0xFF)
            self.set_reg_immediate(registers.B0_HOST_ADDR_S_E,
                                   (hi_flags & 0xFF) | 0x80)
        elif desc.b_bank_mode == 0x01:
            self.set_reg_immediate(registers.B0_HOST_ADDR_S_H,
                                   ((lo >> 8) & 0xFF) | 8)
            self.set_reg_immediate(registers.B0_HOST_ADDR_S_E,
                                   (hi_flags & 0xFF) | 0x80)
        else:
            self.set_reg_immediate(registers.B0_HOST_ADDR_S_H, (lo >> 8) & 0xFF)
            self.set_reg_immediate(registers.B0_HOST_ADDR_S_E, hi_flags & 0xFF)
        self.set_reg_immediate(registers.B0_CLK_DELAY, 0x60)
        for r in (registers.B0_HOST_ADDR_E_L, registers.B0_HOST_ADDR_E_H,
                  registers.B0_HOST_ADDR_E_E):
            self.set_reg_immediate(r, 0xFF)
        if desc.b_op == 0x01:
            self.write_data_dma(desc.dw_size, desc.p_data)
        else:
            self.read_data_dma(desc.dw_size)

    def load_motor_table(self, b_addr: int, p_data) -> None:
        if p_data is None:
            return
        w_addr_lo = ((b_addr << 0x0E) | 0x3000) & 0xFFFF
        w_addr_hi_flags = (b_addr >> 2) & 0xFFFF
        upload = TableDesc(b_op=1, b_bank_mode=0, w_addr_lo=w_addr_lo,
                           w_addr_hi_flags=w_addr_hi_flags, dw_size=0x2000,
                           b_flag=self.t_cmd_chunk(), p_data=p_data)
        self.write_table_block(upload)
        term = TableDesc(b_op=0, b_bank_mode=0, w_addr_lo=w_addr_lo,
                         w_addr_hi_flags=w_addr_hi_flags, dw_size=2,
                         b_flag=self.t_cmd_chunk(), p_data=b"\x00" * 4)
        self.write_table_block(term)
        self.set_reg_immediate(registers.B0_MOTOR_TBL_ADDR, b_addr & 0xFF)

    def write_motor_table(self, table: bytes) -> None:
        self.set_reg_immediate(registers.B0_CLK_DELAY, 0x60)
        sync = TableDesc(
            b_op=0, b_bank_mode=0,
            w_addr_lo=self.dw_motor_table_addr & 0xFFFF,
            w_addr_hi_flags=(self.dw_motor_table_addr >> 0x10) & 0xFFFF,
            dw_size=2, b_flag=0x0C, p_data=b"\x00" * 4)
        self.write_table_block(sync)
        dma = self.dw_motor_table_dma_addr
        self.set_reg_immediate(registers.B0_HOST_ADDR_S_L, dma & 0xFF)
        self.set_reg_immediate(registers.B0_HOST_ADDR_S_H, (dma >> 8) & 0xFF)
        self.set_reg_immediate(registers.B0_HOST_ADDR_S_E, (dma >> 0x10) & 0xFF)
        self.set_reg_immediate(registers.B0_HOST_ADDR_E_L, 0xFF)
        self.set_reg_immediate(registers.B0_HOST_ADDR_E_H, 0xFF)
        self.set_reg_immediate(registers.B0_HOST_ADDR_E_E, 0x3F)
        self.set_reg_immediate(registers.B0_MOTOR_TBL_ADDR,
                               self.dw_motor_table_addr & 0xFF)
        self.write_data_dma(0x2000, table)

    def build_speed_table(self, p_phase_timing_a, p_phase_timing_b, w_dpi,
                          w_entry_limit) -> None:
        if self.n_state < 2:
            self.open_chip()
        if self.n_state == FW_SCANNING:
            self.set_reg_immediate(registers.B0_TRIGGER, 0)
        if w_dpi == 0x4B0:
            uv1 = ((w_entry_limit + 0x19) >> 4) << 4
            tmp_8 = 1
            uv2 = ((w_entry_limit + 0x11) >> 4) << 4
        else:
            uv1 = (((w_entry_limit + 0x19) * 600 // w_dpi) >> 4) << 4
            tmp_8 = 600 // w_dpi
            uv2 = ((w_entry_limit + 0x19) * 600 // w_dpi) & 0xFFF0
        tmp_14 = uv2
        trace("valid_pixel=%d", tmp_14)
        table_size = ((uv1 + 10) * 6 + (((uv1 + 10) * 6) // 0xF0) * 0x10)
        tmp_c = bytearray(table_size * 2)
        uv2 = 0
        tmp_2c = 0
        while tmp_2c <= (tmp_14 & 0xFFFF) // 0x28:
            if tmp_2c < (tmp_14 & 0xFFFF) // 0x28:
                for uv4 in range(0x28):
                    off = uv4 * 0xC + tmp_2c * 0x200
                    b = (p_phase_timing_b[uv2 * 6:]
                         if uv2 * 6 < len(p_phase_timing_b)
                         else b"\x00\x00\x00\x00\x00\x00")
                    a = (p_phase_timing_a[uv2 * 6:]
                         if uv2 * 6 < len(p_phase_timing_a)
                         else b"\x00\x00\x00\x00\x00\x00")
                    tmp_c[off + 0] = b[0]
                    tmp_c[off + 1] = b[1]
                    tmp_c[off + 4] = b[2]
                    tmp_c[off + 5] = b[3]
                    tmp_c[off + 8] = b[4]
                    tmp_c[off + 9] = b[5]
                    tmp_c[off + 2] = a[0]
                    tmp_c[off + 3] = a[1]
                    tmp_c[off + 6] = a[2]
                    tmp_c[off + 7] = a[3]
                    tmp_c[off + 10] = a[4]
                    tmp_c[off + 11] = a[5]
                    if w_entry_limit <= uv2:
                        tmp_c[off + 2] = 0x2000 & 0xFF
                        tmp_c[off + 3] = (0x2000 >> 8) & 0xFF
                        tmp_c[off + 6] = 0x2000 & 0xFF
                        tmp_c[off + 7] = (0x2000 >> 8) & 0xFF
                        tmp_c[off + 10] = 0x2000 & 0xFF
                        tmp_c[off + 11] = (0x2000 >> 8) & 0xFF
                    if (uv4 % (tmp_8 & 0xFFFF) == (tmp_8 & 0xFFFF) - 1
                            and uv2 < w_entry_limit):
                        uv2 += 1
                    if tmp_2c == 0 and uv4 < (tmp_8 & 0xFFFF) * 4:
                        uv2 = 0
            else:
                for tmp_30 in range((tmp_14 & 0xFFFF) % 0x28):
                    off = tmp_30 * 0xC + tmp_2c * 0x200
                    b = (p_phase_timing_b[uv2 * 6:]
                         if uv2 * 6 < len(p_phase_timing_b)
                         else b"\x00\x00\x00\x00\x00\x00")
                    a = (p_phase_timing_a[uv2 * 6:]
                         if uv2 * 6 < len(p_phase_timing_a)
                         else b"\x00\x00\x00\x00\x00\x00")
                    tmp_c[off + 0] = b[0]
                    tmp_c[off + 1] = b[1]
                    tmp_c[off + 4] = b[2]
                    tmp_c[off + 5] = b[3]
                    tmp_c[off + 8] = b[4]
                    tmp_c[off + 9] = b[5]
                    tmp_c[off + 2] = a[0]
                    tmp_c[off + 3] = a[1]
                    tmp_c[off + 6] = a[2]
                    tmp_c[off + 7] = a[3]
                    tmp_c[off + 10] = a[4]
                    tmp_c[off + 11] = a[5]
                    if w_entry_limit <= uv2:
                        tmp_c[off + 2] = 0x2000 & 0xFF
                        tmp_c[off + 3] = (0x2000 >> 8) & 0xFF
                        tmp_c[off + 6] = 0x2000 & 0xFF
                        tmp_c[off + 7] = (0x2000 >> 8) & 0xFF
                        tmp_c[off + 10] = 0x2000 & 0xFF
                        tmp_c[off + 11] = (0x2000 >> 8) & 0xFF
                    if (tmp_30 % (tmp_8 & 0xFFFF)
                            == (tmp_8 & 0xFFFF) - 1
                            and uv2 < w_entry_limit):
                        uv2 += 1
                    if tmp_2c == 0 and tmp_30 < (tmp_8 & 0xFFFF) * 4:
                        uv2 = 0
            tmp_2c += 1
        self.dw_speed_table_addr = (0x300000
                                    - (table_size + 0x4FFF & 0xFFFFC000))
        w_addr_lo = self.dw_speed_table_addr & 0xFFFF
        w_addr_hi = (self.dw_speed_table_addr >> 0x10) & 0xFFFF
        upload = TableDesc(b_op=1, b_bank_mode=0, w_addr_lo=w_addr_lo,
                           w_addr_hi_flags=w_addr_hi,
                           dw_size=table_size * 2,
                           b_flag=self.t_cmd_chunk(), p_data=bytes(tmp_c))
        self.write_table_block(upload)
        term = TableDesc(b_op=0, b_bank_mode=0, w_addr_lo=w_addr_lo,
                         w_addr_hi_flags=w_addr_hi, dw_size=2,
                         b_flag=self.t_cmd_chunk(), p_data=b"\x00" * 4)
        self.write_table_block(term)
        self.set_reg_immediate(registers.B0_SHADING_ADDR,
                               (self.dw_speed_table_addr >> 0x0E) & 0xFF)

    def build_sine_tables(self, dw_speed: int) -> None:
        line_time = (self.read_status_reg(0x0E) << 8)
        line_time = (line_time | self.read_status_reg(0x0D)) << 8
        line_time = (line_time | self.read_status_reg(0x0C)) + 1
        speed = ((dw_speed & 0xFFFF) * line_time) // 0x4B0
        self.set_reg_immediate(registers.B0_FIXED_SPD_L, speed & 0xFF)
        self.set_reg_immediate(registers.B0_FIXED_SPD_H, (speed >> 8) & 0xFF)
        dst = bytearray(0x2000)
        motor_tables.build_phase_tables(dst, count=(speed & 0xFFFF) + 0x9C4,
                                        period=speed & 0xFFFF, steps=0x14)
        cfg = bytearray(0x24)
        cfg[0], cfg[1], cfg[2], cfg[3], cfg[0x23] = 0, 1, 0, 0x3C, 0x3C
        self.set_motor_phase_timing(bytes(cfg))
        upload = TableDesc(b_op=1, b_bank_mode=0, w_addr_lo=0xF000,
                           w_addr_hi_flags=0x2F, dw_size=0x2000,
                           b_flag=self.t_cmd_chunk(), p_data=bytes(dst))
        self.write_table_block(upload)
        term = TableDesc(b_op=0, b_bank_mode=0, w_addr_lo=0xF000,
                         w_addr_hi_flags=0x2F, dw_size=2,
                         b_flag=self.t_cmd_chunk(), p_data=b"\x00" * 4)
        self.write_table_block(term)
        self.set_reg_immediate(registers.B0_MOTOR_TBL_ADDR, 0xBF)

    def set_motor_phase_timing(self, cfg: bytes) -> None:
        tmp_18 = 0xFE if cfg[2] == 0x01 else 0xFF
        self.set_reg_immediate(registers.B0_MOTOR_CUR_CTRL, 1)
        if cfg[1] == 0 or cfg[0] == 0:
            self.set_reg_immediate(registers.B0_PWM_CUR_CTRL, 0)
            for mask in (tmp_18 & 8, tmp_18 & 9, tmp_18 & 1, 0):
                self.set_reg_buffered(registers.B0_CUR_TBL_A, cfg[3])
                self.set_reg_buffered(registers.B0_CUR_TBL_B, cfg[0x23])
                self.set_reg_buffered(registers.B0_PHASE_TBL1, mask)
        elif cfg[0] == 0x02:
            raise NotImplementedError(
                "set_motor_phase_timing mode 0x02 (microstep) not supported")
        elif cfg[0] == 0x06:
            raise NotImplementedError(
                "set_motor_phase_timing mode 0x06 (sinusoidal) not supported")
        elif cfg[0] == 0x0E:
            raise NotImplementedError(
                "set_motor_phase_timing mode 0x0E (sinusoidal) not supported")
        if cfg[1] == 0:
            self.set_reg_immediate(registers.B0_MOTOR_CUR_CTRL, 0)
        else:
            self.set_reg_immediate(registers.B0_MOTOR_CUR_CTRL, cfg[0])

    # ---- scan setup (set_scan_param group) ----

    def set_scan_line_timing(self, dpi, line_pixels, time_offset,
                             minimal_timing):
        tmp_24 = 0x1FF  # ACC_STEP
        tmp_28 = 0x14   # MAX_SPD
        tmp_30 = 0xFF   # DEC_STEP
        tmp_8 = 0x1FF   # SCAN_ACC
        tmp_14 = 0xFF   # SCAN_DEC
        tmp_20 = 0x14   # FIX_SCAN_STEP
        tmp_1c = 10     # BACKTRACK
        tmp_10 = 10     # RESTART
        if minimal_timing == 1:
            tmp_24 = tmp_30 = tmp_8 = tmp_14 = 1
            tmp_20 = tmp_28 = 0
            self.dw_scan_line_timing_mode = 0
        elif dpi < 0x12D:
            if dpi == 300:
                tmp_8, tmp_14, tmp_1c, tmp_10 = 0x78, 0x28, 0x10, 0x10
            elif dpi == 0x32:
                tmp_8, tmp_14, tmp_1c, tmp_10 = 0x96, 0x14, 0x10, 0x10
            elif dpi == 0x4B:
                tmp_8, tmp_14, tmp_1c, tmp_10 = 0x96, 0x14, 0x10, 0x10
            elif dpi == 0x96:
                tmp_8, tmp_14, tmp_1c, tmp_10 = 0x96, 0x28, 0x10, 0x10
        elif dpi == 600:
            tmp_8, tmp_14, tmp_1c, tmp_10 = 200, 100, 200, 200
        elif dpi == 0x4B0:
            tmp_8, tmp_14, tmp_1c, tmp_10 = 200, 200, 200, 200
        elif dpi == 0x960:
            tmp_8, tmp_14, tmp_1c, tmp_10 = 100, 100, 100, 100
        if (line_pixels & 0xFFFF) < tmp_24 + tmp_28 + tmp_30 + tmp_8:
            if (line_pixels & 0xFFFF) <= tmp_8:
                tmp_3c = 1
            else:
                tmp_3c = (line_pixels & 0xFFFF) - tmp_8
            tmp_20 = tmp_3c
            tmp_24 = tmp_30 = 1
            tmp_28 = 0
            self.dw_scan_line_timing_mode = 0
        else:
            uv1 = ((line_pixels & 0xFFFF) - tmp_24) - tmp_30 - tmp_8
            if uv1 <= tmp_20:
                tmp_40 = 0
            else:
                tmp_40 = (uv1 - tmp_20) & 0xFFFF
            tmp_28 = tmp_40
            self.dw_scan_line_timing_mode = 1
        iv2 = (time_offset + tmp_24 + tmp_28 + tmp_30 + tmp_8
                  + tmp_20 + tmp_14 + 2)
        self.set_reg_immediate(registers.B0_ACC_STEP_L, tmp_24 & 0xFF)
        self.set_reg_immediate(registers.B0_ACC_STEP_H, (tmp_24 >> 8) & 0xFF)
        self.set_reg_immediate(registers.B0_MAX_SPD_L, tmp_28 & 0xFF)
        self.set_reg_immediate(registers.B0_MAX_SPD_H, (tmp_28 >> 8) & 0xFF)
        self.set_reg_immediate(registers.B0_MAX_SPD_E, 0)
        self.set_reg_immediate(registers.B0_DEC_STEP, tmp_30 & 0xFF)
        self.set_reg_immediate(0xAE, 0)  # B0_HOST_ADDR_S_L|B0_OFFSET_BLU_P
        self.set_reg_immediate(0xAF, 0)  # |B0_AFE_REG1
        self.set_reg_immediate(registers.B0_SCAN_ACC_L, tmp_8 & 0xFF)
        self.set_reg_immediate(registers.B0_SCAN_ACC_H, (tmp_8 >> 8) & 0xFF)
        self.set_reg_immediate(registers.B0_FIX_SCAN_STEP_L, tmp_20 & 0xFF)
        self.set_reg_immediate(0x8A, (tmp_20 >> 8) & 0xFF)  # FIX_SCAN_STEP_H
        self.set_reg_immediate(registers.B0_SCAN_DEC, tmp_14 & 0xFF)
        self.set_reg_immediate(registers.B0_BACKTRACK_L, tmp_1c & 0xFF)
        self.set_reg_immediate(registers.B0_BACKTRACK_H, 0)
        self.set_reg_immediate(registers.B0_RESTART_L, tmp_10 & 0xFF)
        self.set_reg_immediate(registers.B0_RESTART_H, 0)
        self.set_reg_immediate(registers.B0_BACKHOME_L, 100)
        self.set_reg_immediate(registers.B0_BACKHOME_H, 0)
        self.set_reg_immediate(registers.B0_IMAGE_STEP_L, iv2 & 0xFF)
        self.set_reg_immediate(registers.B0_IMAGE_STEP_H, (iv2 >> 8) & 0xFF)
        self.set_reg_immediate(registers.B0_IMAGE_STEP_E, (iv2 >> 16) & 0xFF)
        return (tmp_8, tmp_14)

    def set_adc_clock_timing(self, cfg):
        if len(cfg) < 9:
            cfg = cfg + b"\x00" * (9 - len(cfg))
        self.set_reg_immediate(registers.B0_AFE_PGARED, cfg[0])
        self.set_reg_immediate(registers.B0_AFE_PGARED | registers.B0_AFE_MUX,
                               cfg[1])
        self.set_reg_immediate(0x08, cfg[2])
        if cfg[6] == 0:
            self.set_reg_immediate(registers.B0_OFFSET_RED_P, cfg[3])
        else:
            self.set_reg_immediate(
                registers.B0_OFFSET_RED_P | registers.B0_AFE_REG1, cfg[3])
        if cfg[7] == 0:
            self.set_reg_immediate(registers.B0_OFFSET_GRN_P, cfg[4])
        else:
            self.set_reg_immediate(
                registers.B0_OFFSET_GRN_P | registers.B0_AFE_REG1, cfg[4])
        if cfg[8] == 0:
            self.set_reg_immediate(registers.B0_OFFSET_BLU_P, cfg[5])
        else:
            self.set_reg_immediate(
                registers.B0_OFFSET_BLU_P | registers.B0_AFE_REG1, cfg[5])

    def write_analog_ctrls(self, cfg):
        if len(cfg) < 0x15:
            cfg = cfg + b"\x00" * (0x15 - len(cfg))
        vals = []
        for off in (9, 0x0B, 0x0D, 0x0F, 0x11, 0x13):
            u = (cfg[off] | (cfg[off + 1] << 8)) & 0xFFFF
            vals.append(u)
        for reg, u in ((registers.B0_RED_EXP_S_L, vals[0]),
                       (registers.B0_RED_EXP_S_H, vals[0]),
                       (registers.B0_RED_EXP_E_L, vals[1]),
                       (registers.B0_RED_EXP_E_H, vals[1]),
                       (registers.B0_GRN_EXP_S_L, vals[2]),
                       (registers.B0_GRN_EXP_S_H, vals[2]),
                       (registers.B0_GRN_EXP_E_L, vals[3]),
                       (registers.B0_GRN_EXP_E_H, vals[3]),
                       (registers.B0_BLU_EXP_S_L, vals[4]),
                       (registers.B0_BLU_EXP_S_H, vals[4]),
                       (registers.B0_BLU_EXP_E_L, vals[5]),
                       (registers.B0_BLU_EXP_E_H, vals[5])):
            self.set_reg_immediate(reg, (u >> (8 * (reg & 1))) & 0xFF)

    def set_motor_phase_profile(self, w_speed, profile):
        profile = bytearray(profile)
        if len(profile) < 0x24:
            profile += b"\x00" * (0x24 - len(profile))
        if profile[0] == 0:
            if w_speed < 0x5DD:
                profile[3], profile[0x23] = (self.b_lamp_current0,
                                            self.b_lamp_current1)
            elif w_speed < 0x9C5:
                profile[3], profile[0x23] = (self.b_lamp_current2,
                                            self.b_lamp_current3)
            elif w_speed < 0x1069:
                profile[3], profile[0x23] = (self.b_lamp_current150_a,
                                            self.b_lamp_current150_b)
            elif w_speed < 0x20D1:
                profile[3], profile[0x23] = (self.b_lamp_current300_a,
                                            self.b_lamp_current300_b)
            elif w_speed < 0x2329:
                profile[3], profile[0x23] = 0x46, 0x46  # 'F'
            elif w_speed < 0x251D:
                profile[3], profile[0x23] = 0x41, 0x41  # 'A'
            elif w_speed < 0x2EE1:
                profile[3], profile[0x23] = 0x3C, 0x3C  # '<'
            elif w_speed < 0x41A1:
                profile[3], profile[0x23] = (self.b_lamp_current600_a,
                                            self.b_lamp_current600_b)
            else:
                profile[3], profile[0x23] = 0x37, 0x37  # '7'
        elif w_speed < 0x1771:
            profile[3], profile[0x23] = 0x46, 0x46  # 'F'
        elif w_speed < 0x2711:
            profile[3], profile[0x23] = 0x3C, 0x3C  # '<'
        elif w_speed < 0x4E21:
            profile[3], profile[0x23] = (self.b_lamp_current1200_a,
                                        self.b_lamp_current1200_b)
        else:
            profile[3], profile[0x23] = (self.b_lamp_current2400_a,
                                        self.b_lamp_current2400_b)
        tmp_8 = 0xFE if profile[2] == 0x01 else 0xFF
        self.set_reg_immediate(registers.B0_MOTOR_CUR_CTRL, 1)
        if profile[1] == 0:
            self.set_reg_immediate(registers.B0_PWM_CUR_CTRL, 0)
            for mask in (tmp_8 & 8, tmp_8 & 9, tmp_8 & 1, 0):
                self._phase_step(profile, mask)
        elif profile[0] == 0:
            self.set_reg_immediate(registers.B0_PWM_CUR_CTRL, 0)
            if w_speed < 0x1389:
                for mask in (tmp_8 & 8, tmp_8 & 9, tmp_8 & 1, 0):
                    self._phase_step(profile, mask)
            else:
                for mask in (tmp_8 & 0x38, tmp_8 & 0x0F, tmp_8 & 0x31,
                             tmp_8 & 0x06):
                    self._phase_step(profile, mask)
        elif profile[0] == 0x02:
            self.set_reg_immediate(registers.B0_PWM_CUR_CTRL, 1)
            for mask in (0x25, 0x07, 0x24, 0x30, 0x2C, 0x0E, 0x2D, 0x39):
                self._phase_step(profile, tmp_8 & mask)
        elif profile[0] == 0x06:
            self.set_reg_immediate(registers.B0_PWM_CUR_CTRL, 2)
            for mask in (0x11, 0x25, 0x03, 0x07, 0x02, 0x24, 0x10, 0x30,
                         0x18, 0x2C, 0x0A, 0x0E, 0x0B, 0x2D, 0x19, 0x39):
                self._phase_step(profile, tmp_8 & mask)
        elif profile[0] == 0x0E:
            raise NotImplementedError(
                "set_motor_phase_profile mode 0x0E (sinusoidal) not supported")
        if profile[1] == 0:
            self.set_reg_immediate(registers.B0_MOTOR_CUR_CTRL, 0)
        else:
            self.set_reg_immediate(registers.B0_MOTOR_CUR_CTRL, profile[0])

    def _phase_step(self, profile, mask):
        """One CUR_TBL_A / CUR_TBL_B / PHASE_TBL1 triple (set_motor_phase_profile)."""
        self.set_reg_buffered(registers.B0_CUR_TBL_A, profile[3])
        self.set_reg_buffered(registers.B0_CUR_TBL_B, profile[0x23])
        self.set_reg_buffered(registers.B0_PHASE_TBL1, mask)

    def set_scan_param(self, pixel_format, w_scan_dpi, w_line_dpi,
                       dw_reserved, n_speed, dw_max_pixels, dw_reserved2,
                       b_analog_mode, b_use_speed_table, n_buffered,
                       b_skip_setup):
        tmp_20 = 0
        tmp_28 = 0
        tmp_60 = 0
        tmp_5c = 0
        tmp_14 = 2
        b_segment_factor = 3
        dw_pixel_multiplier = 0
        dw_max_pixels_adj = 0
        if b_skip_setup == 0:
            self.set_reg_immediate(registers.B0_ACTION, 0)
            self.set_reg_immediate(registers.B0_CLOCK, 0)
            self.set_reg_immediate(registers.B0_TRIGGER, 0)
            _poll_count = 0
            while True:
                _st = self.read_status_reg(1)
                _poll_count += 1
                if _poll_count <= 5 or _poll_count % 20 == 0:
                    trace("engine_idle_poll[%d] status=0x%02x bits5=0x%02x",
                          _poll_count, _st, _st & 0x1F)
                if _st & 0x1F == 0:
                    break
                if _poll_count > 300:
                    raise DriverError(
                        f"set_scan_param: engine idle timeout after {_poll_count} "
                        f"polls, last status=0x{_st:02x}")
                time.sleep(0.05)  # Sleep(0x32)
            self.set_reg_immediate(registers.B0_MOTOR_OPT,
                                   0x50 if b_analog_mode == 0 else 0x51)
            self.set_reg_immediate(registers.B1_PH1_1, 0)
            self.set_reg_immediate(registers.B0_PWR_SAVE_CTRL, 0x27)
            for reg in (registers.B0_MULTI_TG_R, registers.B0_MULTI_TG_G,
                        registers.B0_MULTI_TG_B, registers.B0_MULTI_TG_DUMMY_L,
                        registers.B0_MULTI_TG_DUMMY_H, registers.B0_CCD_DUMMY_L,
                        registers.B0_CCD_DUMMY_H):
                self.set_reg_immediate(reg, 0)
            self.set_reg_immediate(registers.B0_CCD_DUMMY_CYCLES,
                                   self.b_usb_speed_flag)
            self.set_reg_immediate(registers.B0_AFE_CTRL, 1)
            self.set_reg_immediate(registers.B0_AFE_CFG, 0x68)
            self.set_reg_immediate(registers.B0_AFE_MUX, 8)
            self.set_reg_immediate(registers.B0_DIGITAL_CTRL, 0)
            if pixel_format in (1, 8):
                dw_pixel_multiplier = 1
            elif pixel_format == 0x10:
                dw_pixel_multiplier = 2
            elif pixel_format == 0x18:
                dw_pixel_multiplier = 3
            elif pixel_format == 0x30:
                dw_pixel_multiplier = 6
            self.set_reg_immediate(registers.B0_MOTOR_CTRL1, 0)
            if pixel_format in (1, 8):
                b_data_format = 1
            elif pixel_format == 0x10:
                b_data_format = 3
            elif pixel_format == 0x18:
                b_data_format = 0
            elif pixel_format == 0x30:
                b_data_format = 0x32
            else:
                b_data_format = 0
            self.set_reg_immediate(registers.B0_DATA_FMT, b_data_format)
            self.set_reg_immediate(registers.B0_WHITE_FMT, 1)
            if w_scan_dpi < 0x259:
                tmp_20 = 600
                self.set_reg_immediate(registers.B0_GPIO_DIR_LO, 4)
                self.set_reg_immediate(registers.B0_GPIO_LO, 0)
                dw_line_pixels = 0xB04
            else:
                tmp_20 = 0x4B0
                self.set_reg_immediate(registers.B0_GPIO_DIR_LO, 4)
                self.set_reg_immediate(registers.B0_GPIO_LO, 4)
                dw_line_pixels = 0x1428
            dw_line_pixels &= 0xFFFFFFF0
            # wScanDpi/tmp_20, then H_RATIO = ftol(tmp_50 * 32768.0)
            # tmp_54 -> 8192 instead of 16384 (register 0x0020 vs 0x0040).
            tmp_54 = float(w_scan_dpi) / float(tmp_20 & 0xFFFF)
            # [-0x50] in assembly holds ratio = tmp_54 (used for both
            # H_RATIO and the 0xB9/0xBA computation below).
            h_ratio = int(tmp_54 * 32768.0) & 0xFFFF
            self.set_reg_immediate(registers.B0_H_RATIO_L, h_ratio & 0xFF)
            self.set_reg_immediate(registers.B0_H_RATIO_H, (h_ratio >> 8) & 0xFF)
            # Assembly [-0x44] = 32768/H_RATIO (FILD H_RATIO, FDIV 32768,
            # then 1.0/(H_RATIO/32768.0)). Used by the DMA address formula
            # (ftol(8.0*[-0x44]*5.0)) below.
            tmp_48 = 32768.0 / h_ratio
            for i in range(0x10):
                self.set_reg_immediate(0x2B0 + i, 0)
                self.set_reg_immediate(0x2C0 + i, 0)
            self.set_reg_immediate(registers.B1_CCD_PIX_L, dw_line_pixels & 0xFF)
            self.set_reg_immediate(registers.B1_CCD_PIX_H,
                                   (dw_line_pixels >> 8) & 0xFF)
            dw_ccd_pixels = dw_line_pixels
            self.set_reg_immediate(0x169, dw_line_pixels & 0xFF)
            self.set_reg_immediate(0x16A, (dw_line_pixels >> 8) & 0xFF)
            self.set_reg_immediate(0x16B, 0)
            self.set_reg_immediate(registers.B0_LINE_W_L, dw_line_pixels & 0xFF)
            self.set_reg_immediate(registers.B0_LINE_W_H,
                                   (dw_line_pixels >> 8) & 0xFF)
            line_34 = (dw_line_pixels * 3) & 0xFFFF
            self.set_reg_immediate(0x19A, line_34 & 0xFF)
            self.set_reg_immediate(0x19B, (line_34 >> 8) & 0xFF)
            s8 = dw_line_pixels & 0xFF
            if s8 >= 0x80:
                s8 -= 0x100
            for base, mul in ((0x270, -1), (0x273, -1), (0x276, -1),
                              (0x279, 2), (0x27C, -2), (0x27F, -2),
                              (0x282, -2), (0x285, 3)):
                if mul == -1:
                    b0 = (-s8) & 0xFF
                else:
                    b0 = (s8 * mul) & 0xFF
                self.set_reg_immediate(base, b0)
                self.set_reg_immediate(base + 1,
                                       ((dw_line_pixels * mul) >> 8) & 0xFF)
                self.set_reg_immediate(base + 2,
                                       ((dw_line_pixels * mul) >> 0x10) & 0xFF)
            for i in range(0x0C):
                self.set_reg_immediate(0x288 + i, 0)
            self.set_reg_immediate(0xB4, 0)  # B0_CCD_PIX_L|B0_AFE_PGARED
            self.set_reg_immediate(0xB5, 0)  # B0_CCD_PIX_L|B0_AFE_REG4
            lvar9 = int(((line_34 - 1) & 0xFFFF) * tmp_54) & 0xFFFF
            self.set_reg_immediate(0x1B9, lvar9 & 0xFF)
            self.set_reg_immediate(0x1BA, (lvar9 >> 8) & 0xFF)
            self.set_reg_immediate(0x1F4, 0)  # 500
            self.set_reg_immediate(0x1F5, 0)
            self.set_reg_immediate(0x1F6, (dw_max_pixels + 9) & 0xFF)
            self.set_reg_immediate(0x1F7, ((dw_max_pixels + 9) >> 8) & 0xFF)
            self.set_reg_immediate(0x1F8, 0)
            self.set_reg_immediate(0x1F9, 0)
            self.set_reg_immediate(0x1FA, 0x18)
            ccd3 = (dw_ccd_pixels * 3 * b_segment_factor) & 0xFFFFFFFF
            self.set_reg_immediate(0x1FB, ccd3 & 0xFF)
            self.set_reg_immediate(0x1FC, (ccd3 >> 8) & 0xFF)
            self.set_reg_immediate(0x1FD, (ccd3 >> 0x10) & 0xFF)
            self.set_reg_immediate(0x16C, 3)
            self.set_reg_immediate(registers.B1_PH1B_1, 2)
            self.set_reg_immediate(registers.B0_PHTG_EDGE, 0x17)
            self.set_reg_immediate(registers.B0_CLR_PULSE_W, 0)
            self.set_reg_immediate(registers.B0_CLR_INV, 0x55)
            for reg in (registers.B0_TG_R, registers.B0_TG_G,
                        registers.B0_TG_B):
                self.set_reg_immediate(reg, 0)
            addr = (dw_ccd_pixels & 0xFFFF) + 0x300000
            self.set_reg_immediate(0x16D, addr & 0xFF)
            self.set_reg_immediate(0x16E, (addr >> 8) & 0xFF)
            self.set_reg_immediate(0x16F, (addr >> 0x10) & 0xFF)
            if pixel_format < 0x18:
                for reg in range(0x170, 0x176):
                    self.set_reg_immediate(
                        reg, (0xFF, 0xFF, 0x33)[(reg - 0x170) % 3])
            else:
                ft = int(40.0 * tmp_48) & 0xFFFF
                addr = ((dw_ccd_pixels & 0xFFFF) - ft + 0x2FFFF8
                        + (dw_ccd_pixels & 0xFFFF) * 3 * b_segment_factor
                        + (dw_ccd_pixels & 0xFFFF)) & 0xFFFFFFFF
                self.set_reg_immediate(0x170, addr & 0xFF)
                self.set_reg_immediate(0x171, (addr >> 8) & 0xFF)
                self.set_reg_immediate(0x172, (addr >> 0x10) & 0xFF)
                addr = ((dw_ccd_pixels & 0xFFFF) + 0x300000
                        + (dw_ccd_pixels & 0xFFFF) * 3 * b_segment_factor * 2)
                addr &= 0xFFFFFFFF
                self.set_reg_immediate(0x173, addr & 0xFF)
                self.set_reg_immediate(0x174, (addr >> 8) & 0xFF)
                self.set_reg_immediate(0x175, (addr >> 0x10) & 0xFF)
            for reg in range(0x176, 0x191):
                self.set_reg_immediate(
                    reg, (0xFF, 0xFF, 0x33)[(reg - 0x176) % 3])
            for i in range(0x10):
                self.set_reg_immediate(0x260 + i, 0)
            self.set_reg_immediate(0x19E, 0)
            self.set_reg_immediate(0x19F, 0)
            self.set_reg_immediate(0x1A0, 0x30)
            for reg, base, mul in ((0x1A1, 0x300000, 1), (0x1A4, 0x300000, 2),
                                   (0x1A7, 0x2FFFFF, 1),
                                   (0x1AA, 0x2FFFFF, 2),
                                   (0x1AD, 0x2FFFFF, 3)):
                v = ((dw_ccd_pixels & 0xFFFF) * 3 * b_segment_factor * mul
                     + base) & 0xFFFFFFFF
                self.set_reg_immediate(reg, v & 0xFF)
                self.set_reg_immediate(reg + 1, (v >> 8) & 0xFF)
                self.set_reg_immediate(reg + 2, (v >> 0x10) & 0xFF)
            self.set_reg_immediate(0x19C, b_segment_factor)
            self.set_reg_immediate(0x19D, tmp_14)
            for i, reg in enumerate((registers.B0_ADC_CLK0, registers.B0_ADC_CLK1,
                                     registers.B0_ADC_CLK2, registers.B0_ADC_CLK3)):
                self.set_reg_immediate(reg,
                                       (self.dw_afe_adcclk_timing >> (8 * i)) & 0xFF)
            for i, reg in enumerate((0x1F0, 0x1F1, 0x1F2, 0x1F3)):
                self.set_reg_immediate(reg,
                                       (self.dw_afe_adcrs_timing >> (8 * i)) & 0xFF)
            for i, reg in enumerate((0x1EC, 0x1ED, 0x1EE, 0x1EF)):
                self.set_reg_immediate(reg,
                                       (self.dw_afe_adcvs_timing >> (8 * i)) & 0xFF)
            self.set_reg_immediate(0x1E4, 0)
            self.set_reg_immediate(0x1E5, 0)
            self.set_reg_immediate(0x1E6, 0x0F)
            self.set_reg_immediate(0x1E7, 0)
            for i, pos in enumerate((self.w_afe_chalatch_pos,
                                     self.w_afe_chblatch_pos,
                                     self.w_afe_chclatch_pos,
                                     self.w_afe_chdlatch_pos)):
                self.set_reg_immediate(0x160 + i * 2, (pos >> 8) & 0xFF)
                self.set_reg_immediate(0x160 + i * 2 + 1, pos & 0xFF)
            self.set_reg_immediate(0x168, self.b_afe_secondary_fflatch_pos)
            cc = self.dw_ccd_dummy_cycle_timing
            self.set_reg_immediate(registers.B1_PH2B_1, cc & 0xFF)
            self.set_reg_immediate(registers.B1_PHRS_1, (cc >> 8) & 0xFF)
            self.set_reg_immediate(registers.B1_PHCP_1, (cc >> 0x10) & 0xFF)
            self.set_reg_immediate(registers.B1_PHCP_1 | registers.B0_AFE_REG1,
                                   (cc >> 0x18) & 0xFF)
            ph1 = self.dw_ccd_ph1_timing
            self.set_reg_immediate(registers.B1_PH2B_1 | registers.B0_AFE_PGARED,
                                   ph1 & 0xFF)
            self.set_reg_immediate(registers.B1_PH2B_1 | registers.B0_AFE_REG4,
                                   (ph1 >> 8) & 0xFF)
            self.set_reg_immediate(registers.B1_PH2B_1 | registers.B0_AFE_PGARED
                                   | registers.B0_AFE_MUX, (ph1 >> 0x10) & 0xFF)
            self.set_reg_immediate(registers.B1_PH2B_1 | registers.B0_AFE_PGARED
                                   | registers.B0_AFE_MUX | registers.B0_AFE_REG1,
                                   (ph1 >> 0x18) & 0xFF)
            self.set_reg_immediate(registers.B0_PHTG_W, self.b_phtg_pulse_width)
            self.set_reg_immediate(registers.B0_PHTG_WAIT, self.b_phtg_wait_width)
            self.set_reg_immediate(registers.B0_MULTI_TG_DUMMY_H
                                   | registers.B0_AFE_PGARED,
                                   self.b_phtg_timing_adj)
            self.set_reg_immediate(registers.B0_PH1_0, self.b_phtg_timing_setup)
            self.set_reg_immediate(registers.B0_CCD_SETUP,
                                   3 if pixel_format < 0x18 else 7)
            self.set_reg_immediate(registers.B0_TEST_CTRL,
                                   (0x0C if b_use_speed_table == 0 else 0)
                                   | 0xF0)
            self.set_reg_immediate(registers.B0_CCD_PIX_L, self.w_usb_speed & 0xFF)
            self.set_reg_immediate(registers.B0_CCD_PIX_H,
                                   (self.w_usb_speed >> 8) & 0xFF)
            self.set_reg_immediate(registers.B0_ICG_CTRL, 0x17)
            self.set_reg_immediate(registers.B0_LINEART_HI, 0x80)
            self.set_reg_immediate(registers.B0_LINEART_LO, 0x7F)
            dw_max_pixels_adj = dw_max_pixels & 0xFFFF
            if w_line_dpi < 600:
                dw_line_dpi_steps = 600
                n_step_factor = 1
                b_half_step = 0
            elif w_line_dpi < 0x4B0:
                dw_line_dpi_steps = 0x4B0
                n_step_factor = 2
                b_half_step = 2
            elif w_line_dpi < 0x960:
                dw_line_dpi_steps = 0x960
                n_step_factor = 4
                b_half_step = 6
            else:
                dw_line_dpi_steps = 0x960
                n_step_factor = 4
                b_half_step = 6
            n_speed = n_speed * n_step_factor
            dw_motor_line_time = 0
            iv6 = self.w_usb_speed * 3
            dw_motor_line_time = (self.read_status_reg(0x0E) & 0xFF) << 8
            dw_motor_line_time = ((dw_motor_line_time
                                   | (self.read_status_reg(0x0D) & 0xFF)) << 8)
            dw_motor_line_time = (dw_motor_line_time
                                  | (self.read_status_reg(0x0C) & 0xFF)) + 1
            if (iv6 + 0x3F < dw_motor_line_time
                    or dw_motor_line_time < iv6 + 3):
                dw_motor_line_time = iv6 + 0x21
            uv3 = dw_motor_line_time // (
                (dw_line_dpi_steps & 0xFFFF) // w_line_dpi)
            dw_motor_time = uv3
            if (12000 < uv3 < 40000) and b_half_step == 0:
                trace("full_step_to_half_step!!")
                dw_line_dpi_steps = 0x4B0
                b_half_step = 2
                n_speed = n_speed * 2
                dw_motor_time = dw_motor_line_time // (0x4B0 // w_line_dpi)
            trace("---ENDSPEED=%d, dwLinePixelReport", dw_motor_time & 0xFFFF)
            steps, flags = self.set_scan_line_timing(
                dpi=w_line_dpi, line_pixels=n_speed,
                time_offset=int(n_step_factor * dw_reserved2 * 600.0 / w_line_dpi),
                minimal_timing=n_buffered)
            uv3 = dw_motor_time & 0xFFFF
            if dw_motor_time < 0x2EE1:
                if dw_motor_time < 0x7D1:
                    w_fixed_speed_period = dw_motor_time + 5000
                elif dw_motor_time < 0x1389:
                    w_fixed_speed_period = dw_motor_time + 0xDAC
                else:
                    w_fixed_speed_period = dw_motor_time + 1000
            else:
                w_fixed_speed_period = dw_motor_time
            self.set_reg_immediate(registers.B0_FIXED_SPD_L, dw_motor_time & 0xFF)
            self.set_reg_immediate(registers.B0_FIXED_SPD_H,
                                   (dw_motor_time >> 8) & 0xFF)
            dst = bytearray(0x2000)
            motor_tables.build_home_motor_table(
                dst, count=w_fixed_speed_period, period=uv3, steps=steps,
                flags=flags, dpi=w_line_dpi)
            profile = bytearray(0x24)
            profile[0], profile[1], profile[2] = b_half_step, 1, 0
            self.set_motor_phase_profile(uv3, bytes(profile))
            tmp_28 = 0x1000
            tmp_5c = 0x2FF000
            upload = TableDesc(b_op=1, b_bank_mode=0, w_addr_lo=0xF000,
                               w_addr_hi_flags=0x2F, dw_size=0x2000,
                               b_flag=self.t_cmd_chunk(), p_data=bytes(dst))
            self.write_table_block(upload)
            term = TableDesc(b_op=0, b_bank_mode=0, w_addr_lo=0xF000,
                             w_addr_hi_flags=0x2F, dw_size=2,
                             b_flag=self.t_cmd_chunk(), p_data=b"\x00" * 4)
            self.write_table_block(term)
            self.set_reg_immediate(registers.B0_MOTOR_TBL_ADDR,
                                   (tmp_5c >> 0x0E) & 0xFF)
            self.set_reg_immediate(registers.B0_BUF_EMPTY_L, 0)
            self.set_reg_immediate(registers.B0_BUF_EMPTY_H, 0)
            self.set_reg_immediate(registers.B0_PH_RST_EDGE, 0)
            self.set_reg_immediate(registers.B0_CLR_EDGE_W, 0)
        self.set_reg_immediate(registers.B0_ACTION,
                               (self.dw_scan_line_timing_mode & 0xFF) | 0x0C)
        cfg = self._analog_cfg()
        self.set_adc_clock_timing(cfg)
        self.write_analog_ctrls(cfg)
        if b_use_speed_table == 0:
            tmp_60 = 0x2FFFFF - tmp_28
            tmp_5c = tmp_60
        else:
            tmp_60 = self.dw_speed_table_addr - 1
        mul = (dw_max_pixels_adj * (dw_pixel_multiplier & 0xFFFF))
        mul = ((mul & 0xFFFFFFFF) >> 1) & 0xFFFFFFFF
        uv5 = (tmp_60 + ((mul * -3) & 0xFFFFFFFF)) & 0xFFFFFFFF
        uv5 = uv5 >> 6
        self.set_reg_immediate(registers.B0_BUF_FULL_L, uv5 & 0xFF)
        self.set_reg_immediate(registers.B0_BUF_FULL_H, (uv5 >> 8) & 0xFF)
        self.set_motor_table_addr(0, tmp_60, 0)
        time.sleep(0.1)  # Sleep(100)

    def _analog_cfg(self):
        cfg = bytes(self.a_ad_offset_r + self.a_ad_offset_g
                    + [self.b_ad_trim_r, self.b_ad_trim_g, self.b_ad_trim_b])
        return cfg + struct.pack("<6H", self.w_lamp_gain_r, self.w_lamp_offset_r,
                                 self.w_lamp_gain_g, self.w_lamp_offset_g,
                                 self.w_lamp_gain_b, self.w_lamp_offset_b)

    def read_image(self, dw_lines: int) -> bytes:
        if self.n_state != FW_SCANNING:
            raise DriverError("read image requires scanning")
        return self.read_data_dma((dw_lines & 0xFFFF) * self.dw_line_bytes)

    def clear_fifos(self) -> None:
        self.t.send_command(0x05, b"\x00\x00\x00\x00")
        self.t.send_command(0xC0, b"\x00\x00\x00\x00")

    def usb_command_write(self) -> None:
        if self.n_state < FW_SCANNING:
            return
        self.read_data_dma(2)
        self.set_reg_immediate(registers.B0_TRIGGER, 0)
        self.set_reg_immediate(registers.B0_ACTION, 0)
        self.set_reg_immediate(registers.B0_CLOCK, 0)
        self.clear_fifos()

    def read_status_reg(self, selector: int) -> int:
        self.set_reg_immediate(registers.B0_STATUS, selector)
        self.t.send_command(4, b"\x8b" * 4, sub=0x8B)  # set_dma_address(0, 0x8b)
        return self.t.read_command(7, b"\x8b" * 4, sub=0)[0]

    def read_status_selector(self, selector: int) -> int:
        v = self.read_status_reg(0)
        trace("at_home=%x, return=%x", v, v & 0x10)
        return v & 0x10

    def read_status_cmd(self) -> int:
        return self.t.read_command(7, b"\x8b" * 4, sub=0)[0]

    def check_function_key(self) -> int:
        trace("< check_function_key()")
        if self.n_state != FW_SCAN_READY:
            raise DriverError(
                "check_function_key if (state!=scan-ready)")
        self.set_reg_immediate(registers.B0_GPIO_DIR_LO, 0)
        self.set_reg_immediate(registers.B0_GPIO_LO, 0xFF)
        self.set_reg_immediate(0x98, 0)
        self.set_reg_immediate(registers.B0_GPIO_HI, 0xFF)
        self.set_reg_immediate(registers.B0_STATUS, 2)
        self.set_dma_address(0, 0x8B)
        status = self.read_status_cmd()
        mapped = (0xFF - (status & 0xFF)) & 0x37
        trace("GPIO=%x, function_key=%x", status, mapped)
        trace(">0 check_function_key()")
        key = mapped - 1
        if key == 0:
            return 1
        if key == 2:
            return 2
        if key == 4:
            return 4
        if key == 0x10:
            return 8
        if key == 0x20:
            return 0x10
        return 0

    # ---- chip power ----

    def open_chip(self) -> None:
        logger.debug("open_chip: state %d -> %d", self.n_state, FW_SCAN_READY)
        for magic in OPEN_MAGIC:
            self.t.send_command(0x90, magic)
        self.n_state = FW_SCAN_READY

    def close_chip(self) -> None:
        logger.debug("close_chip: state %d", self.n_state)
        for magic in CLOSE_MAGIC:
            self.t.send_command(0x90, magic)

    def set_lamp(self, b_on: int) -> None:
        if self.n_state < 2:
            raise DriverError("set_lamp if (state!=open)")
        if self.n_state > 2:
            self.set_reg_immediate(registers.B0_TRIGGER, 0)
        self.set_reg_immediate(0x99, 0x30)  # B0_LAMP_PWM
        self.set_reg_immediate(0x90, 0xFF)  # B0_LAMP0
        self.set_reg_immediate(0x91, 0x00)  # B0_LAMP1
        if b_on == 0:
            exp_end = 0x65
        else:
            exp_end = 0x07D1
        for base in (0xB8, 0xBC, 0xC0):  # RED/GRN/BLU exposure
            self.set_reg_immediate(base, 100)
            self.set_reg_immediate(base + 1, 0)
            self.set_reg_immediate(base + 2, exp_end & 0xFF)  # E_L
            self.set_reg_immediate(base + 3, exp_end >> 8)    # E_H

    # ---- status / motor sync ----

    def wait_unit_ready(self, timeout: float = 25.0,
                        poll_sleep: float = 0.1) -> None:
        if self.n_state < 2:
            raise DriverError("wait_unit_ready: not scan-ready")
        deadline = time.monotonic() + timeout
        while True:
            if self.read_status_reg(1) & 0x1F == 0:
                return
            if time.monotonic() >= deadline:
                raise DriverTimeoutError("wait_unit_ready failed: engine not idle")
            time.sleep(poll_sleep)

    # ---- motor control ----

    def _poll_motor_idle(self, timeout: float = 25.0,
                         poll_sleep: float = 0.05) -> None:
        deadline = time.monotonic() + timeout
        while True:
            if self.read_status_reg(1) & 0x1F == 0:
                return
            if time.monotonic() >= deadline:
                raise DriverTimeoutError("motor not idle")
            time.sleep(poll_sleep)

    def wait_motor_idle(self, timeout: float = 25.0,
                        poll_sleep: float = 0.05) -> None:
        self.set_reg_immediate(registers.B0_TRIGGER, 0)
        self._poll_motor_idle(timeout, poll_sleep)

    def start_motor(self, b_direction: int, w_speed: int) -> None:
        logger.debug("start_motor direction=%d speed=%d", b_direction, w_speed)
        self.set_reg_immediate(registers.B0_TRIGGER, 0)
        self.dw_motor_speed = 0x5DC
        self.set_reg_immediate(registers.B0_FIXED_SPD_L, 0x68)
        self.set_reg_immediate(registers.B0_FIXED_SPD_H, 0x10)
        self.set_reg_immediate(registers.B0_MOTOR_OPT, 0x51)
        self.set_reg_immediate(registers.B0_MOTOR_CTRL1, 0)
        if b_direction == 0:
            self.set_reg_immediate(registers.B0_ACC_STEP_L, 1)
            self.set_reg_immediate(registers.B0_ACC_STEP_H, 0)
            self.set_reg_immediate(registers.B0_DEC_STEP, 1)
            sv1 = w_speed * 2 - 2
        else:
            self.set_reg_immediate(registers.B0_ACC_STEP_L, 0xFF)
            self.set_reg_immediate(registers.B0_ACC_STEP_H, 1)
            self.set_reg_immediate(registers.B0_DEC_STEP, 0x96)
            sv1 = w_speed * 2 - 0x295
        self.set_reg_immediate(registers.B0_MAX_SPD_L, sv1 & 0xFF)
        self.set_reg_immediate(registers.B0_MAX_SPD_H, (sv1 >> 8) & 0xFF)
        self.set_reg_immediate(registers.B0_MAX_SPD_E, 0)
        self.set_reg_immediate(registers.B0_IMAGE_STEP_L, 0x58)
        self.set_reg_immediate(registers.B0_IMAGE_STEP_H, 0x1B)
        self.set_reg_immediate(registers.B0_IMAGE_STEP_E, 0)
        self.write_motor_table(bytes(0x2000))
        self.set_reg_immediate(registers.B0_ACTION,
                               0x31 if b_direction == 0 else 0x01)
        self.set_reg_immediate(registers.B0_TRIGGER, 1)
        self.wait_unit_ready()
        self.n_state = FW_SCAN_READY

    def back_home(self, b_direction: int) -> None:
        self.set_lamp(0)
        self.motor_home_cmd(b_direction)

    def motor_home_cmd(self, b_direction: int) -> None:
        if (self.read_status_selector(0) & 0xFF) != 0:
            return
        cfg = bytearray(0x24)
        # feed set_motor_phase_timing's phase currents (pPhaseCfg[3]/[0x23]) so
        # (motor phases energized but no torque -> buzz without moving).
        cfg[3] = self.b_lamp_current0
        cfg[0x23] = self.b_lamp_current1
        self.set_motor_phase_timing(bytes(cfg))
        trace("back_home_speed=%d", self.w_back_speed)
        table = bytearray(0x2000)
        motor_tables.build_home_motor_table(
            table, count=5000, period=self.w_back_speed, steps=0x1FF,
            flags=0xFF, dpi=0x4B)
        self.load_motor_table(0, bytes(table))
        self.write_table_block(TableDesc(b_op=0, b_bank_mode=0, w_addr_lo=0,
                                         w_addr_hi_flags=0, dw_size=2,
                                         b_flag=self._table_dma_chunk,
                                         p_data=b"\x00\x00"))
        move = bytearray(0x13)
        move[0] = 0
        move[1] = 2
        move[2] = 1
        move[3] = 0
        move[4], move[5] = 0x70, 0x17          # FIXED_SPD 0x1770
        move[6], move[7], move[8], move[9] = 0, 0, 0, 0  # MAX_SPD (mode 2)
        move[0xA], move[0xB] = 0, 0            # MOTOR_CTRL1
        move[0xC], move[0xD] = 0xFF, 0x01      # ACC_STEP 0x01FF
        move[0xE] = 0xFF                       # DEC_STEP
        move[0xF] = 0x50                       # MOTOR_OPT
        move[0x10] = b_direction               # wait flag
        # stacked [0x11] byte is 0xC0. Match it (was 0x17 from an 18-byte move).
        move[0x11] = 0xC0
        self.llf_motor_move(move)

    def _wait_llf_state(self, target: int, timeout: float = 25.0,
                        poll_sleep: float = 0.1) -> None:
        deadline = time.monotonic() + timeout
        while (self.read_status_selector(0) & 0xFF) != target:
            if time.monotonic() >= deadline:
                raise DriverTimeoutError(
                    f"motor move wait for state 0x{target:02X} failed")
            time.sleep(poll_sleep)

    def llf_motor_move(self, p_move) -> None:
        m = bytes(p_move)
        if len(m) < 0x13:
            m += b"\x00" * (0x13 - len(m))
        logger.debug("llf_motor_move mode=0x%02X dir_flag=0x%02X", m[1], m[0x10])
        self.wait_motor_idle()
        self.set_reg_immediate(registers.B0_ACC_STEP_L, m[0xC] & 0xFF)
        acc_step = m[0xC] | (m[0xD] << 8)
        self.set_reg_immediate(registers.B0_ACC_STEP_H, (acc_step >> 8) & 0xFF)
        self.set_reg_immediate(registers.B0_DEC_STEP, m[0xE] & 0xFF)
        self.set_reg_immediate(registers.B0_FIXED_SPD_L, m[4] & 0xFF)
        fixed_speed = m[4] | (m[5] << 8)
        self.set_reg_immediate(registers.B0_FIXED_SPD_H,
                               (fixed_speed >> 8) & 0xFF)
        self.set_reg_immediate(registers.B0_MOTOR_OPT,
                               m[2] | m[3] | m[0xF])
        self.set_reg_immediate(registers.B0_MOTOR_CTRL1,
                               m[0xA] | m[0xB])
        if m[1] == 0x02:
            tmp_14 = 2
            tmp_10 = 30000
        else:
            tmp_14 = 1
            tmp_10 = (m[6] | (m[7] << 8)
                        | (m[8] << 16) | (m[9] << 24))
            if m[1] == 0x01:
                tmp_14 = 0x11
        if m[1] == 0x03:
            tmp_14 |= 0x83
        self.set_reg_immediate(registers.B0_PWR_SAVE_CTRL,
                               m[0x11] | 0x17 | m[0x12])
        self.set_reg_immediate(registers.B0_MAX_SPD_L, tmp_10 & 0xFF)
        self.set_reg_immediate(registers.B0_MAX_SPD_H, (tmp_10 >> 8) & 0xFF)
        self.set_reg_immediate(registers.B0_MAX_SPD_E, (tmp_10 >> 16) & 0xFF)
        if m[0] == 0x01:
            tmp_14 |= 0x20
        self.set_reg_immediate(registers.B0_ACTION, tmp_14)
        self.set_reg_immediate(registers.B0_TRIGGER, 1)
        if m[0x10] == 0x01:
            if m[1] == 0x02:
                target = {0: 0x10, 4: 0x20, 8: 0x30}.get(m[3])
                if target is not None:
                    self._wait_llf_state(target)
            else:
                self._poll_motor_idle()

    def calculate_scan_param(self, b_direction: int, dw_distance: int,
                             b_wait: int) -> int:
        cfg = bytearray(0x24)
        cfg[3] = 0x46
        cfg[0x23] = 0x46
        self.set_motor_phase_timing(bytes(cfg))
        table = bytearray(0x2000)
        motor_tables.build_phase_tables(table, count=5000, period=3000,
                                        steps=0x1FF)
        self.load_motor_table(0, bytes(table))
        move = bytearray(18)
        move[0] = 0
        move[1] = b_direction & 0xFF
        move[2] = 1
        move[3] = 0
        move[4], move[5] = 5000 & 0xFF, (5000 >> 8) & 0xFF
        move[0xA], move[0xB] = 0, 0
        move[0xF] = 0x50
        if dw_distance < 0x2FF:
            move[0] = 1
            move[0xC], move[0xD], move[0xE] = 0x01, 0x00, 0x01
            n_step = 1 if dw_distance == 2 else dw_distance - 2
        else:
            move[0xC], move[0xD], move[0xE] = 0xFF, 0x01, 0xFF
            n_step = 1 if dw_distance == 0x2FE else dw_distance - 0x2FE
        move[6] = n_step & 0xFF
        move[7] = (n_step >> 8) & 0xFF
        move[8] = (n_step >> 16) & 0xFF
        move[9] = (n_step >> 24) & 0xFF
        move[0x10] = b_wait & 0xFF
        self.llf_motor_move(move)
        return 1

    def scan_start(self) -> None:
        if self.n_state != FW_SCAN_READY:
            raise DriverError("scan start requires scan-ready")
        self.set_reg_immediate(registers.B0_TRIGGER, 1)
        logger.debug("scan_start: state %d -> %d", self.n_state, FW_SCANNING)
        self.n_state = FW_SCANNING

    def stop_scan(self) -> None:
        self.set_reg_immediate(registers.B0_ACTION, 0)
        self.set_reg_immediate(registers.B0_CLOCK, 0)
        self.set_reg_immediate(registers.B0_TRIGGER, 0)
        self._poll_motor_idle()
        self.set_reg_immediate(registers.B1_PH1_1, 0)
        self.set_reg_immediate(registers.B0_PWR_SAVE_CTRL, 0x17)
        if self.n_dram_test_flag:
            self.dram_test()
            self.n_dram_test_flag = 0
        logger.debug("stop_scan: state %d -> %d", self.n_state, FW_SCAN_READY)
        self.n_state = FW_SCAN_READY

    # ---- open / close / reset ----

    def open_connection(self, services) -> None:
        if self.n_state > FW_CONNECTED:
            raise DriverError("connection open: already open")
        if self.n_state == FW_IDLE:
            self.t.open()
            trace("Attach USB device ok")
        self.clear_fifos()
        self.n_state = FW_CONNECTED
        self.b_chip_open = 1
        trace("Create Device ok")

    def close_connection(self) -> None:
        self.t.close()
        self.n_state = FW_IDLE

    def init_chip(self) -> None:
        trace("before REG80 reset")
        self.set_reg_immediate(0x80, 0xFF)
        trace("REG80 reset ok")
        trace("before Open scan chip")
        self.open_chip()
        trace("Open scan chip ok!")
        for v in (0xF1, 0xA5, 0x91, 0x81, 0xF0):
            self.set_reg_immediate(registers.B0_SDRAM, v)
        self.set_reg_immediate(registers.B0_ACTION, 0)
        self.set_reg_immediate(registers.B0_CLOCK, 0)
        self.set_reg_immediate(registers.B0_TRIGGER, 0)
        self._poll_motor_idle()
        self.set_reg_immediate(0x5F, 1)
        self.set_reg_immediate(0xCD, 0)
        self.set_reg_immediate(0x5F, 0)
        self.set_reg_immediate(0x94, 0x27)
        self.n_state = FW_SCAN_READY
        trace("init_chip success!")

    def dram_test(self) -> None:
        pattern = bytes(range(0x40))

        def set_window() -> None:
            self.set_reg_immediate(registers.B0_CLK_DELAY, 0x60)
            for r in (registers.B0_HOST_ADDR_S_L, registers.B0_HOST_ADDR_S_H,
                      registers.B0_HOST_ADDR_S_E):
                self.set_reg_immediate(r, 0)
            self.set_reg_immediate(registers.B0_HOST_ADDR_E_L, 0xFF)
            self.set_reg_immediate(registers.B0_HOST_ADDR_E_H, 0xFF)
            self.set_reg_immediate(registers.B0_HOST_ADDR_E_E, 0x3F)

        set_window()
        self.write_data_dma(0x40, pattern)
        set_window()
        data = self.read_data_dma(0x40)
        if data != pattern:
            trace("DRAM Test error!!")
            raise DriverError("DRAM test: DRAM test error")

    def init_defaults(self) -> None:
        self.dw_line_bytes = 0
        self.b_ad_trim_r = 0
        self.b_ad_trim_g = 1
        self.b_ad_trim_b = 1
        self.a_ad_offset_r = [0x28, 0x28, 0x28]
        self.a_ad_offset_g = [0x78, 0x28, 0x32]
        self.dw_init_timing1 = 60000
