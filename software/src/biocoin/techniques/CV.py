########################################################################################################################
#
# BioCoin Cyclic Voltammetry (CV) Technique
#
# Implements the cyclic voltammetry (CV) measurement protocol for the BioCoin device.
# Handles configuration, execution, BLE data parsing, and conversion of streamed bytes to current readings.
# Built on top of the BaseTechnique class.
#
# Written By:
#   - Drew Hall (DrewHall@ucsd.edu)
#   - Risab Sankar (rsankar@ucsd.edu)
#   - Tyler Hack (thack@ucsd.edu)
#
# Known Issues:
#   - No retry logic if BLE write or notification fails
#
# Revision History:
#   - 17 Aug 2025: Initial implementation of CV technique
#
# Notes:
#   - Should we reconstruct the voltage vector or just send it over BLE?
#
########################################################################################################################
import asyncio
import logging
import struct

import numpy as np

from biocoin.device import BioCoinDevice
from biocoin.techniques.base_technique import BaseTechnique


class CyclicVoltammetry(BaseTechnique):
    """
    Implements the cyclic voltammetry (CV) technique using the BioCoin device.

    Parameters:
        - device:  Connected BioCoin device
    """

    def __init__(self, device: BioCoinDevice):
        super().__init__(device)
        self.V: np.ndarray | None = None

    def calc_voltage_vector(
        self, E_start: float, E_vertex1: float, E_vertex2: float, E_step: float, cycles: int = 1
    ) -> np.ndarray:
        """
        Construct the CV voltage vector

        Parameters:
            - E_start (float): Starting potential (mV)
            - E_vertex1 (float): First vertex potential (mV)
            - E_vertex2 (float): Second vertex potential (mV)
            - E_step (float): Potential step size (mV), must be > 0
            - cycles (int): Number of cycles to perform, must be ≥ 1

        Returns:
            - np.ndarray: Voltage vector for the CV measurement
        """
        # The DAC has a 12-bit resolution. Quantize all the potentials to the nearest 12-bit step.
        FSR = 2400 - 200  # Full-scale range of the 12-bit DAC
        bits = 12  # 12-bit DAC
        LSB = FSR / (2**bits - 1)  # Size of one LSB in mV
        quantize = lambda v: np.floor(v / LSB) * LSB

        if abs(E_step) < LSB:
            raise ValueError(f'|E_step| must be > {LSB:.6f} mV')
        if cycles < 1:
            raise ValueError('cycles must be ≥ 1')

        # Round to the nearest LSB
        ### Note: The firmware does not quantize these values, so we don't do it here to ensure consistency
        #E_start = quantize(E_start)
        #E_vertex1 = quantize(E_vertex1)
        #E_vertex2 = quantize(E_vertex2)
        #E_step = quantize(abs(E_step))

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
        if not np.isclose(cur, E_start, atol=0.5 * LSB):
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
        Pack and send the CV configuration to the device

        Parameters:
            - processing_interval: Time between data processing interrupts (s)
            - max_current:  Maximum current (µA)
            - E_start: Start potential (mV)
            - E_vertex1: First vertex potential (mV)
            - E_vertex2: Second vertex potential (mV)
            - E_step: Potential step size (mV), must be > 0
            - pulse_width: Duration per step (ms), must be > 0
            - channel: Channel number (0-3)
        """
        logging.info('Sending CV technique parameters...')

        if processing_interval <= 0 or processing_interval < pulse_width / 1000.0:
            raise ValueError('processing_interval must be > 0 annd >= pulse_width')
        if not (0 < max_current <= 3000):
            raise ValueError('max_current must be > 0 and ≤ 3,000 µA')
        if E_step <= 0.537 or E_step > 2200:
            raise ValueError('E_step must be between 0.537 mV and 2.2 V')
        if pulse_width <= 3 or pulse_width > 300000:
            raise ValueError('pulse_width must be between 3 ms and 300 s')
        if channel not in {0, 1, 2, 3}:
            raise ValueError('channel must be an integer between 0 and 3')
        if not ((E_vertex1 <= E_start <= E_vertex2) or (E_vertex2 <= E_start <= E_vertex1)):
            raise ValueError('E_start must be bounded between E_vertex1 and E_vertex2')
        if abs(E_vertex2 - E_vertex1) > 2200:
            raise ValueError('E_vertex2 and E_vertex1 must be within 2200 mV of each other')
        # Bounds for potentials
        for name, V in [('E_start', E_start), ('E_vertex1', E_vertex1), ('E_vertex2', E_vertex2)]:
            if not (-2200 <= V <= 2200):
                raise ValueError(f'{name} must be between -2.2 and +2.2 V')

        # Store reconstructed voltage vector
        self.V = self.calc_voltage_vector(E_start, E_vertex1, E_vertex2, E_step)
        self.duration = len(self.V) * pulse_width / 1000.0  # convert ms to seconds

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
        payload = struct.pack(
            '<BfffffffB',
            self.Technique.CV,
            processing_interval,
            max_current,
            E_start,
            E_vertex1,
            E_vertex2,
            E_step,
            pulse_width,
            channel,
        )

        await self.device.write_technique_config(payload)
        await self.assert_config_ok()

    async def run(self) -> np.ndarray:
        """
        Start the CV measurement and return an array of [voltage (mV), current (µA)]

        Parameters:

        Returns:
            - np.ndarray: 2D array with columns [voltage (mV), current (µA)]
        """
        max_polls = 10
        poll_interval = min(0.1 * self.duration, 2)

        try:
            await self.start()  # start notifications
            await self.device.write_ctrl_command(self.Command.START)
            await asyncio.sleep(self.duration)

            # Poll up to max_polls for completion
            for _ in range(1, max_polls + 1):
                if await self.is_done():
                    break
                await asyncio.sleep(poll_interval)
            else:
                # If loop didn't break (not finished), force stop and raise
                await self.device.write_ctrl_command(self.Command.STOP)
                raise TimeoutError(f'Device did not finish after {self.duration + max_polls * poll_interval:.3f}s')
        finally:
            # Ensures notifications are stopped regardless of outcome
            await self.stop()

        # Collect results from the data queue
        results: list[float] = []
        try:
            while True:
                results.append(self.data_queue.get_nowait())
        except asyncio.QueueEmpty:
            pass

        logging.info(f'Received {len(results)} data points from CV.')
        return np.column_stack((self.V, np.asarray(results, dtype=float)))

    async def notification_handler(self, _: int, data: bytes) -> None:
        """
        Parse incoming CV BLE data as 4-byte floats

        Parameters:
            - data: Raw byte stream from BLE
        """
        self.byte_buffer.extend(data)
        logging.debug(f'Received: {data}')

        # Process the byte buffer in chunks of 4 bytes (little-endian float)
        while len(self.byte_buffer) >= 4:
            chunk = self.byte_buffer[:4]
            self.byte_buffer[:4] = []
            value = struct.unpack('<f', chunk)[0]
            self.data_queue.put_nowait(value)
            logging.debug(f'Received CV value: {value}')
