"""
Temperature sensing technique implementation.
"""

import struct
from dataclasses import dataclass
from typing import ClassVar

import numpy as np

from biocoin.device import BiocoinDevice
from biocoin.techniques.base_technique import BaseTechnique
from utils.logging_util import get_logger

logger = get_logger(__name__)


@dataclass(frozen=True, slots=True)
class TempConfig:
    """
    Configuration payload fields for temperature sensing.
    """

    _CHANNEL_MAP: ClassVar[dict[int, int]] = {
        0: 0x1B,  # VAFE4
        1: 0x1D,  # VAFE1
        2: 0x06,  # AIN2
    }

    sampling_interval: float
    processing_interval: float
    channel: int

    def validate(self) -> None:
        """
        Validate configuration constraints before sending to firmware.

        Parameters:
            - None
        Returns:
            - None
        """
        if self.sampling_interval <= 0:
            raise ValueError('sampling_interval must be > 0')
        if self.processing_interval <= 0:
            raise ValueError('processing_interval must be > 0')
        if self.channel not in self._CHANNEL_MAP:
            raise ValueError('channel must be 0, 1, or 2')

    def to_payload(self, technique_id: int) -> bytes:
        """
        Pack this configuration into firmware payload bytes.

        Parameters:
            - technique_id (int): Technique enum value.
        Returns:
            - bytes: Packed payload bytes.
        """
        # Struct layout (packed, little-endian) with a leading technique ID byte:
        # <B f f B
        #  ^  ^ ^ ^
        #  |  | | +-- channel (uint8) -> AD5940 ADCMUXP code (0->0x1B VAFE4, 1->0x1D VAFE1, 2->0x06 AIN2)
        #  |  | +---- processing_interval (float)
        #  |  +------ sampling_interval (float)
        #  +--------- technique ID (uint8)
        return struct.pack(
            '<BffB',
            technique_id,
            self.sampling_interval,
            self.processing_interval,
            self._CHANNEL_MAP[self.channel],
        )


class Temperature(BaseTechnique[float]):
    """
    Implements the temperature (TEMP) technique using the Biocoin device.

    """

    def __init__(self, device: BiocoinDevice):
        """
        Initialize the TEMP technique.

        Parameters:
            - device (BiocoinDevice): The connected Biocoin device
        Returns:
            - None
        """
        super().__init__(device)

    async def configure(self, sampling_interval: float, processing_interval: float, channel: int) -> None:
        """
        Pack and send the TEMP configuration to the device.

        Parameters:
            - sampling_interval (float): Time between samples in seconds
            - processing_interval (float): Time between when samples are processed in seconds
            - channel (int): Channel number (0-3)
        Returns:
            - None
        """
        logger.info('Sending TEMP technique parameters...')

        _config = TempConfig(
            sampling_interval=sampling_interval,
            processing_interval=processing_interval,
            channel=channel,
        )
        _config.validate()

        self.sampling_interval = _config.sampling_interval
        payload = _config.to_payload(self.Technique.TEMP)

        await self.device.write_technique_config(payload)
        await self.assert_config_ok()

    async def run(self, duration: int = 15) -> np.ndarray:
        """
        Start the TEMP measurement and return the measured temperature.

        Parameters:
            - duration (int): Time to run the measurement (in seconds). Default is 15 seconds.

        Returns:
            - np.ndarray: 2D array with columns [time (s), temperature (degC)]
        """
        results = await self._run_fixed_duration(duration)

        logger.info(f'Received {len(results)} data points from Temp.')

        # Build time vector using sampling interval
        t = np.arange(len(results), dtype=float) * self.sampling_interval
        return np.column_stack((t, np.asarray(results, dtype=float)))

    async def notification_handler(self, _: int, data: bytes) -> None:
        """
        Parse incoming TEMP BLE data as 4-byte floats.

        Parameters:
            - sender (int): BLE sender ID (unused)
            - data (bytes): Raw byte stream from BLE
        Returns:
            - None
        """
        self._ingest_float32_samples(data, label='Temp')


