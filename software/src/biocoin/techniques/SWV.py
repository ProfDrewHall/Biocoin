########################################################################################################################
#
# BioCoin Square Wave Voltammetry (SWV) Technique
#
# Implements the SWV measurement protocol for the BioCoin device. Handles configuration, execution, BLE data parsing, 
# and conversion of streamed bytes to current readings. Built on top of the BaseTechnique class.
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
#   - 1 Dec 2025: Initial implementation of SWV technique
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


class SquareWaveVoltammetry(BaseTechnique):
    """
    Implements the square wave voltammetry (SWV) technique using the BioCoin device.

    SWV applies a staircase baseline (Estart→Estop in Estep increments). At each step a pulse of amplitude Epulse (mV) and 
    width PulseWidth (ms) is superimposed; the device returns a (typically differential) current sample per step. For 
    plotting, we associate each returned current with the pulse-peak potential: E_base + Epulse.

    Parameters:
        - device (BioCoinDevice): Connected BioCoin device

    Attributes:
        - V (np.ndarray): Voltage vector (mV) aligned one-to-one with returned current samples
        - duration (float): Estimated run time (s)
    """

    def __init__(self, device: BioCoinDevice):
        super().__init__(device)
        self.V: np.ndarray | None = None

    def calc_voltage_vector(self, E_start: float, E_stop: float, E_step: float) -> np.ndarray:
        """
        Construct the SWV voltage vector (pulse-peak potentials per step)

        Parameters:
            - E_start (float): Starting baseline potential (mV)
            - E_stop  (float): Ending baseline potential (mV)
            - E_step  (float): Baseline increment per step (mV), magnitude > 0

        Returns:
            - np.ndarray: Vector of pulse-peak potentials E_base + Epulse (mV), one per step
        """
        # Quantize to DAC steps (same convention as CV)
        # The DAC has a 12-bit resolution. Quantize all the potentials to the nearest 12-bit step.
        FSR = 2400 - 200  # Full-scale range of the 12-bit DAC
        bits = 12  # 12-bit DAC
        LSB = FSR / (2**bits - 1)  # Size of one LSB in mV
        quantize = lambda v: np.floor(v / LSB) * LSB

        #E_start = quantize(E_start)
        #E_stop = quantize(E_stop)
        #E_step = abs(quantize(E_step))

        if E_step == 0:
            raise ValueError('Estep quantizes to 0 mV; increase Estep.')

        V = self._segment(E_start, E_stop, E_step)
        return V.astype(float, copy=False)

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
        Pack and send the SWV configuration to the device

        Parameters:
            - processing_interval (float): Seconds between MCU processing interrupts (s)
            - max_current (float): Maximum expected current (µA), (0, 3000]
            - E_start (float): Start baseline potential (mV), within [-2200, 2200]
            - E_stop (float): Stop baseline potential (mV), within [-2200, 2200]
            - E_amplitude (float): Pulse amplitude (mV), > 0
            - E_step (float): Baseline step magnitude (mV), > 0
            - pulse_period (float): Period per step (ms), must be ≥ PulseWidth
            - channel (int): Working electrode channel (0-3)

        Returns:
            - None
        """
        logging.info('Sending SWV technique parameters...')

        # Basic sanity checks
        if processing_interval <= 0:
            raise ValueError('processing_interval must be > 0')
        if not (0 < max_current <= 3000):
            raise ValueError('max_current must be > 0 and ≤ 3000 µA')
        for name, V in [('E_start', E_start), ('E_stop', E_stop)]:
            if not (-2200 <= V <= 2200):
                raise ValueError(f'{name} must be between -2200 and +2200 mV')
        if E_step == 0:
            raise ValueError('E_step must be nonzero')
        if E_amplitude <= 0:
            raise ValueError('E_pulse must be > 0')
        if pulse_period <= 3 or pulse_period > 300000:
            raise ValueError('pulse_period must be between 3 ms and 300,000 ms')
        if processing_interval < ((pulse_period/2) / 1000.0):
            raise ValueError('processing_interval must be ≥ pulse_width(in seconds)')
        if channel not in {0, 1, 2, 3}:
            raise ValueError('channel must be an integer between 0 and 3')

        # Build voltage vector for plotting/results alignment
        self.V = self.calc_voltage_vector(E_start, E_stop, E_step)
        self.duration = len(self.V) * (pulse_period / 1000.0)

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
        payload = struct.pack(
            '<BfffffffB',
            self.Technique.SWV,
            processing_interval,
            max_current,
            E_start,
            E_stop,
            E_amplitude,
            E_step,
            pulse_period,
            channel,
        )

        await self.device.write_technique_config(payload)
        await self.assert_config_ok()

    async def run(self) -> np.ndarray:
        """
        Start the SWV measurement and return an array of [V,I] points

        Returns:
            - np.ndarray: 2D array with columns [pulse-peak potential (mV), current (µA)]
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

        logging.info(f'Received {len(results)} data points from SWV.')

        # Pairwise differences: [a,b,c,d,...] -> [a-b, c-d, ...]
        num_pts = len(results) // 2
        if len(results) % 2 != 0:
            logging.warning('Odd number of SWV samples received; dropping last sample.')
        diffs = (
            np.asarray(results[: 2 * num_pts], dtype=float)[0::2]
            - np.asarray(results[: 2 * num_pts], dtype=float)[1::2]
        )
        return np.column_stack((self.V, diffs))

    async def notification_handler(self, _: int, data: bytes) -> None:
        """
        Parse incoming SWV BLE data as 4-byte floats (little-endian)

        Parameters:
            - data (bytes): Raw byte stream from BLE
        """
        self.byte_buffer.extend(data)
        logging.debug(f'Received: {data}')

        # Process the byte buffer in chunks of 4 bytes
        while len(self.byte_buffer) >= 4:
            chunk = self.byte_buffer[:4]
            del self.byte_buffer[:4]
            value = struct.unpack('<f', chunk)[0]
            self.data_queue.put_nowait(value)
            logging.debug(f'Received SWV value: {value}')
