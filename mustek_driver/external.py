import logging
import time

logger = logging.getLogger("mustek_driver")


def trace(msg: str, *args) -> None:
    if args:
        logger.debug(msg, *args)
    else:
        logger.debug(msg)


def configure_logging(level: int = logging.INFO) -> None:
    """Install a console handler on the mustek_driver logger if none exists."""
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter(
            "%(asctime)s %(name)s %(levelname)s %(message)s"))
        logger.addHandler(handler)
    logger.setLevel(level)
    logger.propagate = False


class DriverError(Exception):
    pass


class DriverTimeoutError(DriverError):
    pass


def sleep(seconds: float) -> None:
    time.sleep(seconds)
