"""
Domain-specific exceptions for Biocoin runner and transport workflows.
"""


class BiocoinError(Exception):
    """
    Base exception for Biocoin-specific failures.

    """


class ConfigError(BiocoinError, ValueError):
    """
    Raised when technique or runner configuration is invalid.

    """


class DeviceStatusError(BiocoinError, RuntimeError):
    """
    Raised when device status indicates a runtime failure.

    """


class TransportError(BiocoinError, RuntimeError):
    """
    Raised for BLE transport, connection, or characteristic access failures.

    """


