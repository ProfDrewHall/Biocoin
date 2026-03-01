"""
Cyclic voltammetry technique implementation.
"""

import struct
from dataclasses import dataclass

import numpy as np

from biocoin.device import BiocoinDevice
from biocoin.techniques.base_technique import BaseTechnique
from biocoin.techniques.validation import (
    CV_DAC_LSB_MV,
    CV_E_STEP_MAX_MV,
    CV_E_STEP_MIN_MV,
    ELECTROCHEM_VERTEX_SPAN_MAX_MV,
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
class CVConfig:
    """
    Configuration payload fields for cyclic voltammetry.
    """

    processing_interval: float
    max_current: float
    E_start: float
    E_vertex1: float
    E_vertex2: float
    E_step: float
    pulse_width: float
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
        if self.processing_interval < self.pulse_width / 1000.0:
            raise ValueError(
                'processing_interval must be >= pulse_width in seconds '
                f'(received processing_interval={self.processing_interval} s, '
                f'pulse_width={self.pulse_width} ms -> {self.pulse_width / 1000.0} s)'
            )
        validate_max_current_uA(self.max_current)
        if self.E_step <= CV_E_STEP_MIN_MV or self.E_step > CV_E_STEP_MAX_MV:
            raise ValueError(
                f'E_step must be between {CV_E_STEP_MIN_MV} and {CV_E_STEP_MAX_MV:.0f} mV '
                f'(received {self.E_step} mV)'
            )
        if self.pulse_width <= PULSE_TIMING_MIN_MS or self.pulse_width > PULSE_TIMING_MAX_MS:
            raise ValueError(
                f'pulse_width must be between {PULSE_TIMING_MIN_MS:.0f} and {PULSE_TIMING_MAX_MS:.0f} ms '
                f'(received {self.pulse_width} ms)'
            )
        validate_channel_0_to_3(self.channel)

        for name, value in [
            ('E_start', self.E_start),
            ('E_vertex1', self.E_vertex1),
            ('E_vertex2', self.E_vertex2),
        ]:
            validate_potential_range(name, value)

        if abs(self.E_vertex2 - self.E_vertex1) > ELECTROCHEM_VERTEX_SPAN_MAX_MV:
            raise ValueError(
                f'E_vertex2 and E_vertex1 must be within {ELECTROCHEM_VERTEX_SPAN_MAX_MV:.0f} mV of each other '
                f'(received |{self.E_vertex2} - {self.E_vertex1}| = {abs(self.E_vertex2 - self.E_vertex1)} mV)'
            )
        if not (
            (self.E_vertex1 <= self.E_start <= self.E_vertex2)
            or (self.E_vertex2 <= self.E_start <= self.E_vertex1)
        ):
            raise ValueError(
                'E_start must be bounded between E_vertex1 and E_vertex2 '
                f'(received E_start={self.E_start}, E_vertex1={self.E_vertex1}, E_vertex2={self.E_vertex2})'
            )

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
        #  ^  ^ ^ ^ ^ ^ ^ ^ ^
        #  |  | | | | | | | channel (uint8)
        #  |  | | | | | | pulse_width (float)
        #  |  | | | | | E_step (float)
        #  |  | | | | E_vertex2 (float)
        #  |  | | | E_vertex1 (float)
        #  |  | | E_start (float)
        #  |  | max_current (float)
        #  | processing_interval (float)
        #  technique ID (uint8)
        return struct.pack(
            '<BfffffffB',
            technique_id,
            self.processing_interval,
            self.max_current,
            self.E_start,
            self.E_vertex1,
            self.E_vertex2,
            self.E_step,
            self.pulse_width,
            self.channel,
        )


class CyclicVoltammetry(BaseTechnique[float]):
    """
    Implements cyclic voltammetry (CV) on the Biocoin device.

    """

    def __init__(self, device: BiocoinDevice):
        super().__init__(device)
        self.V: np.ndarray | None = None

    def calc_voltage_vector(
        self, E_start: float, E_vertex1: float, E_vertex2: float, E_step: float, cycles: int = 1
    ) -> np.ndarray:
        """
        Construct the CV voltage vector.

        Parameters:
            - E_start (float): Starting potential (mV)
            - E_vertex1 (float): First vertex potential (mV)
            - E_vertex2 (float): Second vertex potential (mV)
            - E_step (float): Potential step size (mV), must be > 0
            - cycles (int): Number of cycles to perform, must be >= 1

        Returns:
            - np.ndarray: Voltage vector for the CV measurement
        """
        # The DAC has a 12-bit resolution. Use one LSB to validate minimum step size.
        if abs(E_step) < CV_DAC_LSB_MV:
            raise ValueError(f'|E_step| must be > {CV_DAC_LSB_MV:.6f} mV')
        if cycles < 1:
            raise ValueError('cycles must be >= 1')

        # Note: Firmware does not quantize these values, so host-side config remains unquantized.

        segments: list[np.ndarray] = []
        cur = E_start

        # Construct the different segments in a piecewise manner:
        s = self._segment(cur, E_vertex1, E_step)
        segments.append(s)
        cur = s[-1]  # Use the last value of the segment as the new start -- helps if it overshoots

        # 2) cycles of v1 <-> v2
        for i in range(cycles):
            s = self._segment(cur, E_vertex2, E_step)
            segments.append(s[1:])  # drop first sample to avoid duplicating the endpoint
            cur = s[-1]

            if i < cycles - 1:
                s = self._segment(cur, E_vertex1, E_step)
                segments.append(s[1:])
                cur = s[-1]

        # 3) Final return: go to one step short of start
        if not np.isclose(cur, E_start, atol=0.5 * CV_DAC_LSB_MV):
            s = self._segment(cur, E_start, E_step)
            segments.append(s[1:-1])  # include landing on start
        else:
            segments[-1] = segments[-1][:-1]  # already at start, just drop last point to avoid duplication

        return np.concatenate(segments, dtype=float)

    async def configure(
        self,
        processing_interval: float,
        max_current: float,
        E_start: float,
        E_vertex1: float,
        E_vertex2: float,
        E_step: float,
        pulse_width: float,
        channel: int,
    ) -> None:
        """
        Pack and send the CV configuration to the device.

        Parameters:
            - processing_interval: Time between data processing interrupts (s)
            - max_current:  Maximum current (uA)
            - E_start: Start potential (mV)
            - E_vertex1: First vertex potential (mV)
            - E_vertex2: Second vertex potential (mV)
            - E_step: Potential step size (mV), must be > 0
            - pulse_width: Duration per step (ms), must be > 0
            - channel: Channel number (0-3)
        Returns:
            - None
        """
        logger.info('Sending CV technique parameters...')

        _config = CVConfig(
            processing_interval=processing_interval,
            max_current=max_current,
            E_start=E_start,
            E_vertex1=E_vertex1,
            E_vertex2=E_vertex2,
            E_step=E_step,
            pulse_width=pulse_width,
            channel=channel,
        )

        _config.validate()

        # Store reconstructed voltage vector
        self.V = self.calc_voltage_vector(_config.E_start, _config.E_vertex1, _config.E_vertex2, _config.E_step)
        self.duration = max(len(self.V) * _config.pulse_width / 1000.0, 2)  # convert ms to seconds

        payload = _config.to_payload(self.Technique.CV)

        await self.device.write_technique_config(payload)
        await self.assert_config_ok()

    async def run(self) -> np.ndarray:
        """
        Start the CV measurement and return an array of [voltage (mV), current (uA)].

        Parameters:
            - None
        Returns:
            - np.ndarray: 2D array with columns [voltage (mV), current (uA)]
        """
        if self.V is None:
            raise RuntimeError('CV must be configured before run()')

        results = await self._run_streaming_until_done(self.duration)
        currents = np.asarray(results, dtype=float)
        expected_points = len(self.V)
        if len(currents) != expected_points:
            raise RuntimeError(f'Expected {expected_points} CV points, received {len(currents)}')

        logger.info(f'Received {len(currents)} data points from CV.')
        return np.column_stack((self.V, currents))

    async def notification_handler(self, _: int, data: bytes) -> None:
        """
        Parse incoming CV BLE data as 4-byte floats.

        Parameters:
            - data: Raw byte stream from BLE
        Returns:
            - None
        """
        self._ingest_float32_samples(data, label='CV')


