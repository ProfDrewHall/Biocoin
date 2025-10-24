###################################################################################################
#
# Logging utility module
#
# Colored terminal output to make it easier to see errors
#
# Revision History:
#   - 7 Apr 2023 - Initial coding
#   - 22 June 2025: Refactored to use ColorFormatter and ConsoleColorFormatter
#
# Known Issues:
#
#
# Notes:
#   - Based on: https://gist.github.com/hosackm/654d64760e979280e6fb431af999f489
#   - https://alexandra-zaharia.github.io/posts/make-your-own-custom-color-formatter-with-python-logging/
#
###################################################################################################
import logging
import sys
from datetime import datetime
from logging import LogRecord
from pathlib import Path

from colorama import Fore, Style, init

# Configure logging
logger = logging.getLogger(__name__)


class ColorFormatter(logging.Formatter):
    def __init__(self, fmt: str, datefmt: str | None = None, style: str = '%') -> None:
        super().__init__(fmt=fmt, datefmt=datefmt, style=style)
        self.fmt = fmt

        self.FORMATS: dict[int, str] = {
            logging.DEBUG: Fore.BLUE + self.fmt,
            logging.INFO: self.fmt,
            logging.WARNING: Fore.YELLOW + self.fmt,
            logging.ERROR: Fore.RED + self.fmt,
            logging.CRITICAL: Fore.RED + Style.BRIGHT + self.fmt,
        }

    def format(self, record: LogRecord) -> str:
        """
        Format the log record with color based on the log level.

        Parameters:
            - record: The log record to format

        Returns:
            - A formatted string with color applied based on the log level
        """
        if log_fmt := self.FORMATS.get(record.levelno, self.fmt):
            formatter = logging.Formatter(log_fmt)

            # Clear the format at the end
            formatted = formatter.format(record)
            return formatted + str(Style.RESET_ALL)

        # Default if we don't customize it
        return super().format(record)


class ConsoleColorFormatter(ColorFormatter):
    def format(self, record: LogRecord) -> str:
        # INFO messages: clean, no level prefix or color
        if record.levelno == logging.INFO:
            return record.getMessage()

        # Strip traceback info for console logs
        record.exc_info = None
        record.exc_text = None

        # All other levels: use the base class color logic
        return super().format(record)


def setup_logging(log_dir: Path = Path('./logs'), verbose: bool = False) -> logging.Logger:
    """
    Set up a module-specific logger that writes to both console and file

    Parameters:
        log_dir (Path): Directory to save log files
        verbose (bool): If True, set console log level to DEBUG; otherwise, INFO

    Returns:
        logging.Logger: Configured logger instance
    """
    if logging.getLogger().hasHandlers():
        return logging.getLogger(__name__)  # Avoid reconfiguring

    init(autoreset=True)  # Initialize colorama for colored output

    # Get module name and path info
    module_name = Path(sys.argv[0]).stem
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f'{module_name}_{timestamp}.log'

    # Log to the display
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.DEBUG if verbose else logging.INFO)
    console_handler.setFormatter(ConsoleColorFormatter('%(levelname)s: %(message)s'))

    # Log to a file
    file_handler = logging.FileHandler(log_path)
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(ColorFormatter('%(asctime)-25s | %(levelname)-8s | %(message)s'))

    # Set up the root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)
    root_logger.handlers.clear()
    root_logger.addHandler(file_handler)
    root_logger.addHandler(console_handler)
    root_logger.propagate = False

    # Suppress noisy libraries
    for noisy_logger in ['httpx', 'httpcore', 'openai']:
        logging.getLogger(noisy_logger).setLevel(logging.WARNING)

    return logging.getLogger(__name__)
