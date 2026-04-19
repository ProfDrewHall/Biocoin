"""
Impedance technique implementation.
"""

import struct
from dataclasses import dataclass

import numpy as np

from biocoin.device import BiocoinDevice
from biocoin.techniques.base_technique import BaseTechnique
from biocoin.techniques.validation import (
    ELECTROCHEM_MAX_CURRENT_UA,
    IMP_E_AC_MAX_MV,
    IMP_E_AC_MIN_MV,
)
from utils.logging_util import get_logger

logger = get_logger(__name__)


@dataclass(frozen=True, slots=True)
class ImpedanceConfig:
    """
    Configuration payload fields for impedance measurements.
    """

    sampling_interval: float
    processing_interval: float
    imp_4wire: bool
    ac_coupled: bool
    max_current: float
    E_ac: float
    frequency: float

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
        if self.processing_interval <= 0 or self.processing_interval < self.sampling_interval:
            raise ValueError('processing_interval must be > 0 and >= sampling_interval')
        if not (0 < self.max_current <= ELECTROCHEM_MAX_CURRENT_UA):
            raise ValueError('max_current must be > 0 and <= 3,000 uA')
        if not (IMP_E_AC_MIN_MV < self.E_ac <= IMP_E_AC_MAX_MV):
            raise ValueError('E_ac must be between 0 and 2200 mV')
        if self.frequency <= 0:
            raise ValueError('frequency must be > 0')

    def to_payload(self, technique_id: int) -> bytes:
        """
        Pack this configuration into firmware payload bytes.

        Parameters:
            - technique_id (int): Technique enum value.
        Returns:
            - bytes: Packed payload bytes.
        """
        # Struct layout (packed, little-endian) with a leading technique ID byte:
        # <B f f B B f f f
        #  ^  ^ ^ ^ ^ ^ ^ ^
        #  |  | | | | | | frequency (float)
        #  |  | | | | | E_ac (float)
        #  |  | | | | max_current (float)
        #  |  | | | AC_coupled (uint8)
        #  |  | | IMP_4wire (uint8)
        #  |  | processing_interval (float)
        #  | sampling_interval (float)
        #  technique ID (uint8)
        return struct.pack(
            '<BffBBfff',
            technique_id,
            self.sampling_interval,
            self.processing_interval,
            int(self.imp_4wire),
            int(self.ac_coupled),
            self.max_current,
            self.E_ac,
            self.frequency,
        )


class Impedance(BaseTechnique[tuple[float, float]]):
    """
    Implements the impedance (IMP) technique using the Biocoin device.
    """

    def __init__(self, device: BiocoinDevice):
        super().__init__(device)

    async def configure(
        self,
        sampling_interval: float,
        processing_interval: float,
        IMP_4wire: bool,
        AC_coupled: bool,
        max_current: float,
        E_ac: float,
        frequency: float,
    ) -> None:
        """
        Pack and send the IMP configuration to the device.

        Parameters:
            - sampling_interval: Time between ADC samples (s), must be > 0
            - processing_interval: Time between processing interrupts (s), must be >= sampling_interval
            - IMP_4wire: True for 4-wire, False for 2-wire
            - AC_coupled: True for AC coupling, False for DC coupling
            - max_current: Maximum current (uA), must be > 0 and <= 3000
            - E_ac: AC excitation amplitude (mV), must be between 0 and 2200
            - frequency: Excitation frequency (Hz), must be > 0
        Returns:
            - None
        """
        logger.info('Sending IMP technique parameters...')

        _config = ImpedanceConfig(
            sampling_interval=sampling_interval,
            processing_interval=processing_interval,
            imp_4wire=IMP_4wire,
            ac_coupled=AC_coupled,
            max_current=max_current,
            E_ac=E_ac,
            frequency=frequency,
        )
        _config.validate()

        # Duration is not fixed - depends on frequency and acquisition length.
        # For now, set a conservative bound so run() has a wait time.
        self.duration = max(1.0, 10.0 / _config.frequency)
        payload = _config.to_payload(self.Technique.IMP)

        await self.device.write_technique_config(payload)
        await self.assert_config_ok()

    async def run(self, duration: int = 15) -> np.ndarray:
        """
        Start the IMP measurement and return an array of [magnitude, phase].

        Parameters:
            - duration (int): Time to run the measurement (in seconds). Default is 15 seconds.

        Returns:
            - np.ndarray: 2D array with columns [magnitude (Ohm), phase (deg)]
        """
        results = await self._run_fixed_duration(duration)

        logger.info(f'Received {len(results)} data points from IMP.')
        return np.asarray(results, dtype=float)

    async def notification_handler(self, _: int, data: bytes) -> None:
        """
        Parse incoming IMP BLE data as pairs of 4-byte floats (magnitude, phase).

        Parameters:
            - data: Raw byte stream from BLE
        Returns:
            - None
        """
        self._ingest_float32_pairs(
            data,
            label='IMP',
            second_transform=lambda phase: phase * (180.0 / np.pi),
        )


