"""
Iontophoresis technique implementation.
"""

import asyncio
import struct
from dataclasses import dataclass

import numpy as np

from biocoin.device import BiocoinDevice
from biocoin.techniques.base_technique import BaseTechnique
from utils.logging_util import get_logger

logger = get_logger(__name__)


@dataclass(frozen=True, slots=True)
class IontophoresisConfig:
    """
    Configuration payload fields for iontophoresis.
    """

    current_monitor_interval: float
    stim_current: float
    current_safety_threshold: float

    def validate(self) -> None:
        """
        Validate configuration constraints before sending to firmware.

        Parameters:
            - None
        Returns:
            - None
        """
        if self.current_monitor_interval <= 0:
            raise ValueError('current_monitor_interval must be > 0')
        if self.stim_current <= 0:
            raise ValueError('stim_current must be > 0')
        if self.current_safety_threshold <= 0:
            raise ValueError('current_safety_threshold must be > 0')

    def to_payload(self, technique_id: int) -> bytes:
        """
        Pack this configuration into firmware payload bytes.

        Parameters:
            - technique_id (int): Technique enum value.
        Returns:
            - bytes: Packed payload bytes.
        """
        # Struct layout (packed, little-endian) with a leading technique ID byte:
        # <B f f f
        #  ^  ^ ^ ^
        #  |  | | +-- current_safety_threshold (float, uA) - abort if |I| exceeds this
        #  |  | +---- stim_current (float, uA) - commanded stimulation current
        #  |  +------ current_monitor_interval (float, s) - poll interval for current/status checks
        #  +--------- technique ID (uint8)
        return struct.pack(
            '<Bfff',
            technique_id,
            float(self.current_monitor_interval),
            float(self.stim_current),
            float(self.current_safety_threshold),
        )


class Iontophoresis(BaseTechnique[float]):
    """
    Implements the iontophoresis technique using the Biocoin device.

    Iontophoresis applies a constant current to drive charged molecules into the skin.
    The device monitors the current to ensure it stays within safety thresholds.
    """

    def __init__(self, device: BiocoinDevice):
        """
        Initialize the iontophoresis technique.

        Parameters:
            - device (BiocoinDevice): The connected Biocoin device
        Returns:
            - None
        """
        super().__init__(device)

    async def configure(
        self,
        current_monitor_interval: float,
        stim_current: float,
        current_safety_threshold: float,
    ) -> None:
        """
        Pack and send the iontophoresis configuration to the device.

        Parameters:
            - current_monitor_interval (float): Time between current monitoring samples (seconds)
            - stim_current (float): Stimulation current in microamperes (uA)
            - current_safety_threshold (float): Safety current threshold in microamperes (uA)
        Returns:
            - None
        """
        logger.info('Sending Iontophoresis technique parameters...')

        _config = IontophoresisConfig(
            current_monitor_interval=current_monitor_interval,
            stim_current=stim_current,
            current_safety_threshold=current_safety_threshold,
        )
        _config.validate()

        self.sampling_interval = _config.current_monitor_interval
        payload = _config.to_payload(self.Technique.IONTOPHORESIS)

        await self.device.write_technique_config(payload)
        await self.assert_config_ok()

    async def run(self, duration: int = 15) -> np.ndarray:
        """
        Start iontophoresis, wait for the requested duration, then stop.

        Parameters:
            - duration (int): Time to run the measurement (in seconds). Default is 15 seconds.
        Returns:
            - np.ndarray: Empty (0, 2) array; iontophoresis does not stream data.
        """
        await self.clear_queue()
        await self.device.write_ctrl_command(self.Command.START)

        try:
            await asyncio.sleep(duration)
        finally:
            try:
                await self.device.write_ctrl_command(self.Command.STOP)
            except Exception as exc:
                logger.debug(f'Failed to send STOP after iontophoresis run: {exc}')

        # No streamed data for this technique, but keep the return shape consistent.
        return np.empty((0, 2), dtype=float)


    async def notification_handler(self, _: int, _data: bytes) -> None:
        """
        No notifications for this technique.

        Parameters:
            - _ (int): BLE sender ID (unused).
            - _data (bytes): Raw byte stream from BLE (unused).
        Returns:
            - None
        """
        return None


