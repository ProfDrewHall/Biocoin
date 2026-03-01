"""
Square-wave voltammetry technique implementation.
"""

import struct
from dataclasses import dataclass

import numpy as np

from biocoin.device import BiocoinDevice
from biocoin.techniques.base_technique import BaseTechnique
from biocoin.techniques.pulse_voltammetry import (
    calc_staircase_voltage_vector,
    pairwise_differential_current,
)
from biocoin.techniques.validation import (
    PULSE_TIMING_MAX_MS,
    PULSE_TIMING_MIN_MS,
    validate_channel_0_to_3,
    validate_max_current_uA,
    validate_potential_range,
    validate_processing_interval_positive,
)
from utils.logging_util import get_logger

logger = get_logger(__name__)


@dataclass(frozen=True, slots=True)
class SWVConfig:
    """
    Configuration payload fields for square-wave voltammetry.
    """

    processing_interval: float
    max_current: float
    E_start: float
    E_stop: float
    E_amplitude: float
    E_step: float
    pulse_period: float
    channel: int

    def validate(self) -> None:
        """
        Validate configuration constraints before sending to firmware.

        Parameters:
            - None
        Returns:
            - None
        """
        validate_processing_interval_positive(self.processing_interval)
        validate_max_current_uA(self.max_current)
        for name, value in [('E_start', self.E_start), ('E_stop', self.E_stop)]:
            validate_potential_range(name, value)
        if self.E_step == 0:
            raise ValueError('E_step must be nonzero')
        if self.E_amplitude <= 0:
            raise ValueError('E_amplitude must be > 0')
        if self.pulse_period <= PULSE_TIMING_MIN_MS or self.pulse_period > PULSE_TIMING_MAX_MS:
            raise ValueError('pulse_period must be between 3 ms and 300,000 ms')
        if self.processing_interval < ((self.pulse_period / 2) / 1000.0):
            raise ValueError('processing_interval must be >= pulse_period / 2 (in seconds)')
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
        # <B f f f f f f f B
        #  ^ ^ ^ ^ ^ ^ ^ ^ ^
        #  | | | | | | | | | channel (uint8)
        #  | | | | | | | | PulsePeriod (float)
        #  | | | | | | Estep (float)
        #  | | | | | Eamplitude (float)
        #  | | | | Estop (float)
        #  | | | Estart (float)
        #  | | max_current (float)
        #  | processing_interval (float)
        #  technique ID (uint8)
        return struct.pack(
            '<BfffffffB',
            technique_id,
            self.processing_interval,
            self.max_current,
            self.E_start,
            self.E_stop,
            self.E_amplitude,
            self.E_step,
            self.pulse_period,
            self.channel,
        )


class SquareWaveVoltammetry(BaseTechnique[float]):
    """
    Implements square-wave voltammetry (SWV) on the Biocoin device.

    SWV applies a staircase baseline (Estart -> Estop in Estep increments).
    At each step a pulse of amplitude Epulse (mV) and width PulseWidth (ms) is superimposed;
    the device returns a (typically differential) current sample per step.
    For plotting, we associate each returned current with the pulse-peak potential: E_base + Epulse.
    """

    def __init__(self, device: BiocoinDevice):
        super().__init__(device)
        self.V: np.ndarray | None = None

    def calc_voltage_vector(self, E_start: float, E_stop: float, E_step: float) -> np.ndarray:
        """
        Construct the SWV voltage vector (pulse-peak potentials per step).

        Parameters:
            - E_start (float): Starting baseline potential (mV)
            - E_stop  (float): Ending baseline potential (mV)
            - E_step  (float): Baseline increment per step (mV), magnitude > 0
        Returns:
            - np.ndarray: Vector of pulse-peak potentials E_base + Epulse (mV), one per step
        """
        return calc_staircase_voltage_vector(self._segment, E_start=E_start, E_stop=E_stop, E_step=E_step)

    async def configure(
        self,
        processing_interval: float,
        max_current: float,
        E_start: float,
        E_stop: float,
        E_amplitude: float,
        E_step: float,
        pulse_period: float,
        channel: int,
    ) -> None:
        """
        Pack and send the SWV configuration to the device.

        Parameters:
            - processing_interval: Seconds between MCU processing interrupts (s)
            - max_current: Maximum expected current (uA), (0, 3000]
            - E_start: Start baseline potential (mV), within [-2200, 2200]
            - E_stop: Stop baseline potential (mV), within [-2200, 2200]
            - E_amplitude: Pulse amplitude (mV), > 0
            - E_step: Baseline step magnitude (mV), > 0
            - pulse_period: Period per step (ms), must be >= PulseWidth
            - channel: Working electrode channel (0-3)
        Returns:
            - None
        """
        logger.info('Sending SWV technique parameters...')

        _config = SWVConfig(
            processing_interval=processing_interval,
            max_current=max_current,
            E_start=E_start,
            E_stop=E_stop,
            E_amplitude=E_amplitude,
            E_step=E_step,
            pulse_period=pulse_period,
            channel=channel,
        )
        _config.validate()

        # Build voltage vector for plotting/results alignment
        self.V = self.calc_voltage_vector(_config.E_start, _config.E_stop, _config.E_step)
        self.duration = max(len(self.V) * (_config.pulse_period / 1000.0), 2)

        payload = _config.to_payload(self.Technique.SWV)

        await self.device.write_technique_config(payload)
        await self.assert_config_ok()

    async def run(self) -> np.ndarray:
        """
        Start the SWV measurement and return an array of [V,I] points.

        Parameters:
            - None
        Returns:
            - np.ndarray: 2D array with columns [pulse-peak potential (mV), current (uA)]
        """
        if self.V is None:
            raise RuntimeError('SWV must be configured before run()')

        results = await self._run_streaming_until_done(self.duration)

        logger.info(f'Received {len(results)} data points from SWV.')

        diffs = pairwise_differential_current(results, second_minus_first=True, label='SWV')
        if len(diffs) != len(self.V):
            raise RuntimeError(f'Expected {len(self.V)} SWV points, received {len(diffs)}')
        return np.column_stack((self.V, diffs))

    async def notification_handler(self, _: int, data: bytes) -> None:
        """
        Parse incoming SWV BLE data as 4-byte floats (little-endian).

        Parameters:
            - data (bytes): Raw byte stream from BLE
        Returns:
            - None
        """
        self._ingest_float32_samples(data, label='SWV')


