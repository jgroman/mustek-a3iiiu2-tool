import logging
import struct
from abc import ABC, abstractmethod
from typing import Optional

import usb.core
import usb.util

from mustek_driver.external import DriverError, trace

logger = logging.getLogger(__name__)

VID = 0x055F
PID = 0x040B

# Command opcodes
CMD_DMA_WRITE = 0x02
CMD_DMA_READ = 0x03
CMD_SET_DMA_ADDR = 0x04
CMD_CLEAR_FIFO1 = 0x05
CMD_READ_STATUS = 0x07
CMD_WRITE_REG = 0xB0
CMD_CLR_FIFO2_STOP = 0xC0

EP_BULK_OUT = 0x01
EP_BULK_IN = 0x82
DMA_CHUNK = 0x8000


def build_command_header(opcode: int, payload_len: int, sub: int) -> bytes:
    return struct.pack("<H2xH2xIH2x", opcode & 0xFFFF, sub & 0xFFFF,
                       payload_len, payload_len & 0xFFFF)


class Transport(ABC):

    @abstractmethod
    def send_command(self, opcode: int, payload: bytes = b"", sub: int = 0,
                     payload_len: Optional[int] = None) -> None:
        raise NotImplementedError

    @abstractmethod
    def read_command(self, opcode: int, payload: bytes = b"", sub: int = 0) -> bytes:
        raise NotImplementedError

    @abstractmethod
    def bulk_write(self, data) -> None:
        raise NotImplementedError

    @abstractmethod
    def bulk_read(self, size: int) -> bytes:
        raise NotImplementedError

    @abstractmethod
    def open(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError


class LinuxUsbTransport(Transport):
    """pyusb-based USB transport for the scanner."""

    def __init__(self, dev=None):
        self.dev = dev if dev is not None else self._find()

    @staticmethod
    def _find():
        dev = usb.core.find(idVendor=VID, idProduct=PID)
        if dev is None:
            raise DriverError("No Mustek scanner found (055f:040b).")
        return dev

    def open(self) -> None:
        try:
            self.dev.set_configuration()
        except usb.core.USBError:
            pass
        try:
            usb.util.claim_interface(self.dev, 0)
        except usb.core.USBError as e:
            raise DriverError(
                "Cannot access the scanner USB interface. Run with sudo or "
                "install mustek_driver/udev/90-mustek.rules.") from e

    def close(self) -> None:
        trace("in detach usbpipe")
        try:
            usb.util.release_interface(self.dev, 0)
        except usb.core.USBError:
            pass
        trace("out detach usbpipe")

    def send_command(self, opcode: int, payload: bytes = b"", sub: int = 0,
                     payload_len: Optional[int] = None) -> None:
        logger.debug("send_command opcode=0x%02X sub=0x%02X len=%d",
                     opcode, sub, len(payload))
        self.dev.ctrl_transfer(0x40, 0x04, opcode, sub, payload)

    def read_command(self, opcode: int, payload: bytes = b"", sub: int = 0) -> bytes:
        """Read a vendor control transfer: bmRequestType=0xC0, bRequest=0x04,
        wValue=opcode, wIndex=sub, wLength=len(payload).

        The scanner returns len(payload) bytes.  We allocate a buffer of that
        size and let libusb fill it; there is no extra 16-byte header on the
        wire.
        """
        nbytes = len(payload)
        logger.debug("read_command opcode=0x%02X sub=0x%02X len=%d",
                     opcode, sub, nbytes)
        return bytes(self.dev.ctrl_transfer(0xC0, 0x04, opcode, sub, nbytes))

    def bulk_write(self, data) -> None:
        logger.debug("bulk_write size=%d", len(data))
        self.dev.write(EP_BULK_OUT, bytes(data), timeout=10000)

    def bulk_read(self, size: int) -> bytes:
        logger.debug("bulk_read size=%d", size)
        return bytes(self.dev.read(EP_BULK_IN, size, timeout=10000))
