"""
Logging helpers for consistent app and library output.
"""

import logging
import sys
from datetime import datetime
from logging import LogRecord
from pathlib import Path
from typing import Literal

from colorama import Fore, Style, init


def get_logger(name: str) -> logging.Logger:
    """
    Return a module logger.

    Parameters:
        - name (str): Logger name.
    Returns:
        - logging.Logger: Logger instance.
    """
    return logging.getLogger(name)


class ColorFormatter(logging.Formatter):
    """
    Colorize log records by level for terminal and file handlers.

    """

    def __init__(self, fmt: str, datefmt: str | None = None, style: Literal['%', '{', '$'] = '%') -> None:
        super().__init__(fmt=fmt, datefmt=datefmt, style=style)
        self.fmt = fmt
        self.formats: dict[int, str] = {
            logging.DEBUG: Fore.BLUE + self.fmt,
            logging.INFO: self.fmt,
            logging.WARNING: Fore.YELLOW + self.fmt,
            logging.ERROR: Fore.RED + self.fmt,
            logging.CRITICAL: Fore.RED + Style.BRIGHT + self.fmt,
        }

    def format(self, record: LogRecord) -> str:
        """
        Format a record with level-specific color.

        Parameters:
            - record (LogRecord): Log record to format.
        Returns:
            - str: Formatted log message.
        """
        log_fmt = self.formats.get(record.levelno, self.fmt)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record) + str(Style.RESET_ALL)


class ConsoleColorFormatter(ColorFormatter):
    """
    Suppress noisy prefixes for INFO and tracebacks on console output.

    """

    def format(self, record: LogRecord) -> str:
        if record.levelno == logging.INFO:
            return record.getMessage()

        record.exc_info = None
        record.exc_text = None
        return super().format(record)


def setup_logging(log_dir: Path = Path('./logs'), verbose: bool = False) -> logging.Logger:
    """
    Configure root logging with colored console and timestamped file handlers.

    Parameters:
        - log_dir (Path): Directory for file logs.
        - verbose (bool): Enable debug-level console logs.
    Returns:
        - logging.Logger: Module logger.
    """
    if logging.getLogger().hasHandlers():
        return get_logger(__name__)

    init(autoreset=True)

    module_name = Path(sys.argv[0]).stem
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f'{module_name}_{timestamp}.log'

    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.DEBUG if verbose else logging.INFO)
    console_handler.setFormatter(ConsoleColorFormatter('%(levelname)s: %(message)s'))

    file_handler = logging.FileHandler(log_path)
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(ColorFormatter('%(asctime)-25s | %(levelname)-8s | %(message)s'))

    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)
    root_logger.handlers.clear()
    root_logger.addHandler(file_handler)
    root_logger.addHandler(console_handler)
    root_logger.propagate = False

    for noisy_logger in ['httpx', 'httpcore', 'openai']:
        logging.getLogger(noisy_logger).setLevel(logging.WARNING)

    return get_logger(__name__)


