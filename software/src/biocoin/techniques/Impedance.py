########################################################################################################################
#
# BioCoin Impedance (IMP) Technique
#
# Implements the impedance spectroscopy (IMP) measurement protocol for the BioCoin device.
# Handles configuration, execution, BLE data parsing, and conversion of streamed bytes to magnitude/phase readings.
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
#   - 22 Aug 2025: Initial implementation of IMP technique
#
########################################################################################################################
import asyncio
import logging
import struct

import numpy as np

from biocoin.device import BioCoinDevice
from biocoin.techniques.base_technique import BaseTechnique


class Impedance(BaseTechnique):
    """
    Implements the impedance (IMP) technique using the BioCoin device.

    Parameters:
        - device:  Connected BioCoin device
    """

    def __init__(self, device: BioCoinDevice):
        super().__init__(device)

    async def configure(self, sampling_interval: float, processing_interval: float, IMP_4wire: bool, AC_coupled: bool, max_current: float,
        E_ac: float, frequency: float) -> None:
        """
        Pack and send the IMP configuration to the device

        Parameters:
            - sampling_interval: Time between ADC samples (s), must be > 0
            - processing_interval: Time between processing interrupts (s), must be ≥ sampling_interval
            - IMP_4wire: True for 4-wire, False for 2-wire
            - AC_coupled: True for AC coupling, False for DC coupling
            - max_current: Maximum current (µA), must be > 0 and ≤ 3000
            - E_ac: AC excitation amplitude (mV), must be between 0 and 2200
            - frequency: Excitation frequency (Hz), must be > 0
        """
        logging.info('Sending IMP technique parameters...')

        if sampling_interval <= 0:
            raise ValueError('sampling_interval must be > 0')
        if processing_interval <= 0 or processing_interval < sampling_interval:
            raise ValueError('processing_interval must be > 0 and ≥ sampling_interval')
        if not (0 < max_current <= 3000):
            raise ValueError('max_current must be > 0 and ≤ 3,000 µA')
        if not (0 < E_ac <= 2200):
            raise ValueError('E_ac must be between 0 and 2200 mV')
        if frequency <= 0:
            raise ValueError('frequency must be > 0')

        # Duration is not fixed — depends on frequency and acquisition length.
        # For now, set a conservative bound so run() has a wait time.
        self.duration = max(1.0, 10.0 / frequency)

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
        payload = struct.pack(
            '<BffBBfff',
            self.Technique.IMP,
            sampling_interval,
            processing_interval,
            int(IMP_4wire),
            int(AC_coupled),
            max_current,
            E_ac,
            frequency,
        )

        await self.device.write_technique_config(payload)
        await self.assert_config_ok()

    async def run(self, duration: int = 15) -> np.ndarray:
        """
        Start the IMP measurement and return an array of [magnitude, phase]

        Parameters:
            - duration (int): Time to run the measurement (in seconds). Default is 15 seconds.

        Returns:
            - np.ndarray: 2D array with columns [magnitude (Ohm), phase (deg)]
        """
        await self.start()  # start notifications
        await self.device.write_ctrl_command(self.Command.START)
        await asyncio.sleep(duration)  # Run for specified duration
        await self.device.write_ctrl_command(self.Command.STOP)  # Stop command
        await asyncio.sleep(1)  # Collect any remaining items before turning off notifications
        await self.stop()  # Stop notifications

        # Collect results from the data queue
        results: list[tuple[float, float]] = []
        try:
            while True:
                results.append(self.data_queue.get_nowait())
        except asyncio.QueueEmpty:
            pass

        logging.info(f'Received {len(results)} data points from IMP.')
        return np.asarray(results, dtype=float)

    async def notification_handler(self, _: int, data: bytes) -> None:
        """
        Parse incoming IMP BLE data as pairs of 4-byte floats (magnitude, phase)

        Parameters:
            - data: Raw byte stream from BLE
        """
        self.byte_buffer.extend(data)
        logging.debug(f'Received: {data}')

        # Process the byte buffer in chunks of 8 bytes (2 floats)
        while len(self.byte_buffer) >= 8:
            chunk = self.byte_buffer[:8]
            self.byte_buffer[:8] = []
            magnitude, phase = struct.unpack('<ff', chunk)
            phase = phase * (180.0 / np.pi) 
            self.data_queue.put_nowait((magnitude, phase))
            logging.debug(f'Received IMP value: Mag={magnitude}, Phase={phase}')
