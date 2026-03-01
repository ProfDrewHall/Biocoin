"""
Chronoamperometry technique implementation.
"""

import struct
from dataclasses import dataclass

import numpy as np

from biocoin.device import BiocoinDevice
from biocoin.techniques.base_technique import BaseTechnique
from biocoin.techniques.validation import (
    CA_MAX_CURRENT_UA,
    CA_PULSE_POTENTIAL_MAX_MV,
    CA_PULSE_POTENTIAL_MIN_MV,
    validate_channel_0_to_3,
)
from utils.logging_util import get_logger

logger = get_logger(__name__)


@dataclass(frozen=True, slots=True)
class CAConfig:
    """
    Configuration payload fields for chronoamperometry.
    """

    sampling_interval: float
    processing_interval: float
    max_current: float
    pulse_potential: float
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
        if not (0 < self.max_current <= CA_MAX_CURRENT_UA):
            raise ValueError('max_current must be > 0 and <= 10,000 uA')
        if not (CA_PULSE_POTENTIAL_MIN_MV <= self.pulse_potential <= CA_PULSE_POTENTIAL_MAX_MV):
            raise ValueError('pulse_potential must be between -1000 and +1000 mV')
        validate_channel_0_to_3(self.channel)

    def to_payload(self, technique_id: int) -> bytes:
        """
        Pack this configuration into firmware payload bytes.

        Parameters:
            - technique_id (int): Technique enum value.
        Returns:
            - bytes: Packed payload bytes.
        """
        # Struct layout (packed, little-endian) with a leading technique ID byte:
        # <B f f f f B
        #  ^  ^ ^ ^ ^ ^
        #  |  | | | | +-- channel (uint8)
        #  |  | | | +---- pulse_potential (float, mV)
        #  |  | | +------ max_current (float, uA)
        #  |  | +-------- processing_interval (float, s)
        #  |  +---------- sampling_interval (float, s)
        #  +------------- technique ID (uint8)
        return struct.pack(
            '<BffffB',
            technique_id,
            self.sampling_interval,
            self.processing_interval,
            self.max_current,
            self.pulse_potential,
            self.channel,
        )


class ChronoAmperometry(BaseTechnique[float]):
    """
    Implements the chronoamperometry (CA) technique using the Biocoin device.
    """

    def __init__(self, device: BiocoinDevice):
        """
        Initialize the CA technique.

        Parameters:
            - device (BiocoinDevice): The connected Biocoin device
        Returns:
            - None
        """
        super().__init__(device)

    async def configure(
        self,
        sampling_interval: float,
        processing_interval: float,
        max_current: float,
        pulse_potential: float,
        channel: int,
    ) -> None:
        """
        Pack and send the CA configuration to the device.

        Parameters:
            - sampling_interval (float): Time between samples in seconds
            - processing_interval (float): Time between when samples are processed in seconds
            - max_current (float): Maximum current in microamperes (uA)
            - pulse_potential (float): Pulse potential in millivolts (mV)
            - channel (int): Channel number (1-4)
        Returns:
            - None
        """
        logger.info('Sending CA technique parameters...')

        _config = CAConfig(
            sampling_interval=sampling_interval,
            processing_interval=processing_interval,
            max_current=max_current,
            pulse_potential=pulse_potential,
            channel=channel,
        )
        _config.validate()

        self.sampling_interval = _config.sampling_interval
        payload = _config.to_payload(self.Technique.CA)

        await self.device.write_technique_config(payload)
        await self.assert_config_ok()

    async def run(self, duration: int = 15) -> np.ndarray:
        """
        Start the CA measurement and return a list of measured currents.

        Parameters:
            - duration (int): Time to run the measurement (in seconds). Default is 15 seconds.
        Returns:
            - np.ndarray: 2D array with columns [time (s), current (uA)]
        """
        results = await self._run_fixed_duration(duration)

        logger.info(f'Received {len(results)} data points from CA.')

        # Build time vector using sampling interval
        t = np.arange(len(results)) * self.sampling_interval
        return np.column_stack((t, results))

    async def notification_handler(self, _: int, data: bytes) -> None:
        """
        Parse incoming CA BLE data as 4-byte floats.

        Parameters:
            - sender (int): BLE sender ID (unused)
            - data (bytes): Raw byte stream from BLE
        Returns:
            - None
        """
        self._ingest_float32_samples(data, label='CA')


