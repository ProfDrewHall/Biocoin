########################################################################################################################
#
# BioCoin Open-Circuit Potential (OCP) Technique
#
# Implements the open-circuit potential (OCP) measurement protocol for the BioCoin device.
# Handles configuration, execution, BLE data parsing, and conversion of streamed bytes to OCP readings.
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
#   - 25 August 2025: Initial implementation of OCP technique (mirrors TEMP style)
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


class OpenCircuitPotential(BaseTechnique):
    """
    Implements the open-circuit potential (OCP) technique using the BioCoin device.
    """

    # Logical channel (0–3) → AD5940 ADCMUXP code remap
    _CHANNEL_MAP: dict[int, int] = {
        0: 0x16,  # ADCMUXP_AIN6
        1: 0x19,  # ADCMUXP_VAFE3
        2: 0x10,  # ADCMUXP_AIN0
        3: 0x18,  # ADCMUXP_VAFE2
    }

    def __init__(self, device: BioCoinDevice):
        """
        Initialize the OCP technique

        Parameters:
            - device (BioCoinDevice): The connected BioCoin device
        """
        super().__init__(device)

    async def configure(self, sampling_interval: float, processing_interval: float, channel: int) -> None:
        """
        Pack and send the OCP configuration to the device

        Parameters:
            - sampling_interval (float): Time between samples in seconds
            - processing_interval (float): Time between when samples are processed in seconds
            - channel (int): Logical channel index (0→AIN6, 1→VAFE3, 2→AIN0, 3→VAFE2)
        """
        logging.info('Sending OCP technique parameters...')

        if sampling_interval <= 0:
            raise ValueError('sampling_interval must be > 0')
        if processing_interval <= 0:
            raise ValueError('processing_interval must be > 0')
        if channel not in self._CHANNEL_MAP:
            raise ValueError('channel must be 0, 1, 2, or 3')

        self.sampling_interval = sampling_interval

        # Struct layout (packed, little-endian) with a leading technique ID byte:
        # <B f f B
        #  ^  ^ ^ ^
        #  |  | | +-- channel (uint8) → AD5940 ADCMUXP code (0→AIN6, 1→VAFE3, 2→AIN0, 3→VAFE2)
        #  |  | +---- processing_interval (float)
        #  |  +------ sampling_interval (float)
        #  +--------- technique ID (uint8)
        config = struct.pack(
            '<BffB',
            self.Technique.OCP,
            sampling_interval,
            processing_interval,
            self._CHANNEL_MAP[channel],
        )

        await self.device.write_technique_config(config)
        await self.assert_config_ok()

    async def run(self, duration: int = 15) -> np.ndarray:
        """
        Start the OCP measurement and return the measured voltages

        Parameters:
            - duration (int): Time to run the measurement (in seconds). Default is 15 seconds.

        Returns:
            - np.ndarray: 2D array with columns [time (s), voltage (mV)]
        """
        await self.start()  # Start notifications
        await self.device.write_ctrl_command(self.Command.START)
        await asyncio.sleep(duration)
        await self.device.write_ctrl_command(self.Command.STOP)
        await asyncio.sleep(1)
        await self.stop()

        results: list[float] = []
        try:
            while True:
                results.append(self.data_queue.get_nowait())
        except asyncio.QueueEmpty:
            pass

        logging.info(f'Received {len(results)} data points from OCP.')

        t = np.arange(len(results), dtype=float) * self.sampling_interval
        return np.column_stack((t, np.asarray(results, dtype=float)))

    async def notification_handler(self, _: int, data: bytes) -> None:
        """
        Parse incoming OCP BLE data as 4-byte floats

        Parameters:
            sender (int): BLE sender ID (unused)
            data (bytes): Raw byte stream from BLE
        """
        self.byte_buffer.extend(data)
        logging.debug(f'Received: {data}')

        while len(self.byte_buffer) >= 4:
            chunk = self.byte_buffer[:4]
            del self.byte_buffer[:4]
            value = struct.unpack('<f', chunk)[0]
            self.data_queue.put_nowait(value)
            logging.debug(f'Received OCP value: {value}')