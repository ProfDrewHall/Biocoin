########################################################################################################################
#
# BioCoin Temperature (TEMP) Technique
#
# Implements the temperature monitoring protocol for the BioCoin device. Handles configuration, execution, BLE data 
# parsing, and conversion of streamed bytes to temperature readings. Built on top of the BaseTechnique class.
#
# Written By:
#   - Drew Hall (DrewHall@ucsd.edu)
#   - Risab Sankar (rsankar@ucsd.edu)
#   - Tyler Hack (thack@ucsd.edu)
#
# Known Issues:
#   - Assumes data arrives in 4-byte float chunks (little-endian)
#   - No retry logic if BLE write or notification fails
#
# Revision History:
#   - 25 August 2025: Initial implementation of TEMP technique (mirrors CA style)
#
# Notes:
#   - The duration of the experiment (ie: the pulse length in CA or Run time in OCP, TEMP, IMP, and iontophoresis)
#   is controlled by the host.
#
########################################################################################################################
import asyncio
import logging
import struct

import numpy as np

from biocoin.device import BioCoinDevice
from biocoin.techniques.base_technique import BaseTechnique


class Temperature(BaseTechnique):
    """
    Implements the temperature (TEMP) technique using the BioCoin device.
    """
    # Logical channel (0,1,2) → AD5940 ADCMUXP code remap
    _CHANNEL_MAP: dict[int, int] = {
        0: 0x1B,  # VAFE4
        1: 0x1D,  # VAFE1
        2: 0x06,  # AIN2
    }

    def __init__(self, device: BioCoinDevice):
        """
        Initialize the TEMP technique.

        Parameters:
            - device (BioCoinDevice): The connected BioCoin device
        """
        super().__init__(device)

    async def configure(self, sampling_interval: float, processing_interval: float, channel: int) -> None:
        """
        Pack and send the TEMP configuration to the device

        Parameters:
            - sampling_interval (float): Time between samples in seconds
            - processing_interval (float): Time between when samples are processed in seconds
            - channel (int): Channel number (0-3)
        """
        logging.info('Sending TEMP technique parameters...')

        if sampling_interval <= 0:
            raise ValueError('sampling_interval must be > 0')
        if processing_interval <= 0:
            raise ValueError('processing_interval must be > 0')
        if channel not in self._CHANNEL_MAP:
            raise ValueError('channel must be 0, 1, or 2')

        self.sampling_interval = sampling_interval

        # Struct layout (packed, little-endian) with a leading technique ID byte:
        # <B f f B
        #  ^  ^ ^ ^
        #  |  | | +-- channel (uint8) → AD5940 ADCMUXP code (0→0x1B VAFE4, 1→0x1D VAFE1, 2→0x06 AIN2)
        #  |  | +---- processing_interval (float)
        #  |  +------ sampling_interval (float)
        #  +--------- technique ID (uint8)
        config = struct.pack(
            '<BffB',
            self.Technique.TEMP,
            sampling_interval,
            processing_interval,
            self._CHANNEL_MAP[channel],
        )

        await self.device.write_technique_config(config)
        await self.assert_config_ok()

    async def run(self, duration: int = 15) -> np.ndarray:
        """
        Start the TEMP measurement and return the measured temperature

        Parameters:
            - duration (int): Time to run the measurement (in seconds). Default is 15 seconds.

        Returns:
            - np.ndarray: 2D array with columns [time (s), temperature (°C)]
        """
        await self.start()  # Start notifications
        await self.device.write_ctrl_command(self.Command.START)  # Start command
        await asyncio.sleep(duration)  # Run for specified duration
        await self.device.write_ctrl_command(self.Command.STOP)  # Stop command
        await asyncio.sleep(1)  # Collect any remaining items before turning off notifications
        await self.stop()  # Stop notifications

        # Collect data from queue into list (non-blocking)
        results: list[float] = []
        try:
            while True:
                results.append(self.data_queue.get_nowait())
        except asyncio.QueueEmpty:
            pass

        logging.info(f'Received {len(results)} data points from Temp.')

        # Build time vector using sampling interval
        t = np.arange(len(results), dtype=float) * self.sampling_interval
        return np.column_stack((t, np.asarray(results, dtype=float)))

    async def notification_handler(self, _: int, data: bytes) -> None:
        """
        Parse incoming TEMP BLE data as 4-byte floats

        Parameters:
            - sender (int): BLE sender ID (unused)
            - data (bytes): Raw byte stream from BLE
        """
        # Save incoming bytes
        self.byte_buffer.extend(data)
        logging.debug(f'Received: {data}')

        # Process the byte buffer in chunks of 4 bytes (little-endian float)
        while len(self.byte_buffer) >= 4:
            chunk = self.byte_buffer[:4]
            del self.byte_buffer[:4]
            value = struct.unpack('<f', chunk)[0]
            self.data_queue.put_nowait(value)
            logging.debug(f'Received Temp value: {value} mV')
