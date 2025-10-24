from .device import BioCoinDevice

__all__ = ['BioCoinDevice']

# Optional: define version string
__version__ = '0.1.0'

import logging

logger = logging.getLogger(__name__)
# if not logger.hasHandlers():
#    logging.basicConfig(level=logging.INFO)
