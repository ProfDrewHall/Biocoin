########################################################################################################################
#
# BioCoin Chronoamperometry (CA) Technique
#
# Implements the chronoamperometry (CA) measurement protocol for the BioCoin device.
# Handles configuration, execution, BLE data parsing, and conversion of streamed bytes to current readings.
# Built on top of the BaseTechnique class.
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
#   - 28 June 2025: Initial implementation of CA technique
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


class ChronoAmperometry(BaseTechnique):
    """
    Implements the chronoamperometry (CA) technique using the BioCoin device.
    """

    def __init__(self, device: BioCoinDevice):
        """
        Initialize the CA technique

        Parameters:
            - device (BioCoinDevice): The connected BioCoin device
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
        Pack and send the CA configuration to the device

        Parameters:
            - sampling_interval (float): Time between samples in seconds
            - processing_interval (float): Time between when samples are processed in seconds
            - max_current (float): Maximum current in microamperes (µA)
            - pulse_potential (float): Pulse potential in millivolts (mV)
            - channel (int): Channel number (1-4)

        """
        logging.info('Sending CA technique parameters...')

        if sampling_interval <= 0:
            raise ValueError('sampling_interval must be > 0')
        if processing_interval <= 0:
            raise ValueError('processing_interval must be > 0')
        if not (0 < max_current <= 10_000):
            raise ValueError('max_current must be > 0 and ≤ 10,000 µA')
        if not (-1000 <= pulse_potential <= 1000):
            raise ValueError('pulse_potential must be between -1000 and +1000 mV')
        if channel not in {0, 1, 2, 3}:
            raise ValueError('channel must be an integer between 0 and 3')

        self.sampling_interval = sampling_interval

        # Struct layout (packed, little-endian) with a leading technique ID byte:
        # <B f f f f B
        #  ^  ^ ^ ^ ^ ^
        #  |  | | | | +-- channel (uint8)
        #  |  | | | +---- pulse_potential (float, mV)
        #  |  | | +------ max_current (float, µA)
        #  |  | +-------- processing_interval (float, s)
        #  |  +---------- sampling_interval (float, s)
        #  +------------- technique ID (uint8)
        config = struct.pack(
            '<BffffB', 
            self.Technique.CA,
            sampling_interval,
            processing_interval,
            max_current,
            pulse_potential,
            channel,
        )

        await self.device.write_technique_config(config)
        await self.assert_config_ok()

    async def run(self, duration: int = 15) -> np.ndarray:
        """
        Start the CA measurement and return a list of measured currents

        Parameters:
            - duration (int): Time to run the measurement (in seconds). Default is 15 seconds.

        Returns:
            - np.ndarray: 2D array with columns [time (s), current (uA)]
        """
        await self.start()  # Start notifications
        await self.device.write_ctrl_command(self.Command.START)  # Start command
        await asyncio.sleep(duration)  # Run for specified duration
        await self.device.write_ctrl_command(self.Command.STOP)  # Stop command
        await asyncio.sleep(1)  # Collect any remaining items before turning off notifications
        await self.stop()  # Stop notifications

        # Collect data from queue into list. This is non-blocking and more efficient than using await self.data_queue.get()
        results = []
        try:
            while True:
                results.append(self.data_queue.get_nowait())
        except asyncio.QueueEmpty:
            pass

        logging.info(f'Received {len(results)} data points from CA.')

        # Build time vector using sampling interval
        t = np.arange(len(results)) * self.sampling_interval
        return np.column_stack((t, results))

    async def notification_handler(self, _: int, data: bytes) -> None:
        """
        Parse incoming CA BLE data as 4-byte floats

        Parameters:
            - sender (int): BLE sender ID (unused)
            - data (bytes): Raw byte stream from BLE
        """
        # Save the incoming data to the byte buffer
        self.byte_buffer.extend(data)

        logging.debug(f'Received: {data}')

        # Process the byte buffer in chunks of 4 bytes (little-endian floats)
        while len(self.byte_buffer) >= 4:
            chunk = self.byte_buffer[:4]
            self.byte_buffer[:4] = []
            float_val = struct.unpack('<f', chunk)[0]
            self.data_queue.put_nowait(float_val)
            logging.debug(f'Received CA value: {float_val}')
