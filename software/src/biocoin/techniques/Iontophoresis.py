########################################################################################################################
#
# BioCoin Iontophoresis Technique
#
# Implements the iontophoresis protocol for the BioCoin device. Handles configuration, execution, BLE data parsing, and 
# conversion of streamed bytes to current readings. Built on top of the BaseTechnique class.
#
# Written By:
#   - Drew Hall (DrewHall@ucsd.edu)
#   - Risab Sankar (rsankar@ucsd.edu)
#   - Tyler Hack (thack@ucsd.edu)
#
# Known Issues:
#
# Revision History:
#   - 29 Aug 2025: Initial implementation of iontophoresis technique
#
# Notes:
#   - The duration of the experiment is controlled by the host (us)
#
########################################################################################################################
import asyncio
import logging
import struct

import numpy as np

from biocoin.device import BioCoinDevice
from biocoin.techniques.base_technique import BaseTechnique


class Iontophoresis(BaseTechnique):
    """
    Implements the iontophoresis technique using the BioCoin device.
    """

    def __init__(self, device: BioCoinDevice):
        """
        Initialize the iontophoresis technique.

        Parameters:
            - device (BioCoinDevice): The connected BioCoin device
        """
        super().__init__(device)

    async def configure(self, current_monitor_interval: float, stim_current: float, current_safety_threshold: float) -> None:
        """
        Pack and send the iontophoresis configuration to the device.

        Parameters:
            - current_monitor_interval (float): Time between current monitoring samples (seconds)
            - stim_current (float): Stimulation current in microamperes (µA)
            - current_safety_threshold (float): Safety current threshold in microamperes (µA)
        """
        logging.info('Sending Iontophoresis technique parameters...')

        if current_monitor_interval <= 0:
            raise ValueError('current_monitor_interval must be > 0')
        if stim_current <= 0:
            raise ValueError('stim_current must be > 0')
        if current_safety_threshold <= 0:
            raise ValueError('current_safety_threshold must be > 0')

        self.sampling_interval = current_monitor_interval

        # Struct layout (packed, little-endian) with a leading technique ID byte:
        # <B f f f
        #  ^  ^ ^ ^
        #  |  | | +-- current_safety_threshold (float, µA) — abort if |I| exceeds this
        #  |  | +---- stim_current (float, µA) — commanded stimulation current
        #  |  +------ current_monitor_interval (float, s) — poll interval for current/status checks
        #  +--------- technique ID (uint8)
        config = struct.pack(
            '<Bfff',
            int(self.Technique.IONTOPHORESIS),
            float(current_monitor_interval),
            float(stim_current),
            float(current_safety_threshold),
        )

        await self.device.write_technique_config(config)
        await self.assert_config_ok()

    async def run(self, duration: int = 15, poll_interval: int = 5) -> np.ndarray:
        """
        Start the iontophoresis measurement and return a list of floats.

        Parameters:
            - duration (int): Time to run the measurement (in seconds). Default is 15 seconds.
            - poll_duration (int): Time to poll the status of the current limit (in seconds). Default is 5 seconds.

        Returns:
            - np.ndarray: Empty (0, 2) array; iontophoresis does not stream data.
        """
        #poll_interval = 10

        # No notifications for this technique; just clear any stale queue data.
        #await self.clear_queue()

        await self.device.write_ctrl_command(self.Command.START)  # Start command
        await asyncio.sleep(duration)  # Run for specified duration
        await self.device.write_ctrl_command(self.Command.STOP)  # Stop command   

        # await self.device.write_ctrl_command(self.Command.START)
        # logging.info('Iontophoresis started; polling status...')

        # deadline = asyncio.get_running_loop().time() + float(duration)
        # last_status = None

        # await asyncio.sleep(poll_interval)

        # try:
        #     while True:
        #         status = await self.get_status()
        #         if status != last_status:
        #             logging.debug('Iontophoresis status: %s (%d)', status.name, int(status))
        #             last_status = status

        #         # Handle faults immediately
        #         if self.is_fault_status(status):
        #             await self.device.write_ctrl_command(self.Command.STOP)
        #             if status == self.Status.INVALID_PARAMETERS:
        #                 raise ValueError('Iontophoresis: INVALID_PARAMETERS.')
        #             if status == self.Status.CURRENT_LIMIT_EXCEEDED:
        #                 raise RuntimeError('Iontophoresis aborted: CURRENT_LIMIT_EXCEEDED.')
        #             raise RuntimeError('Iontophoresis aborted due to device ERROR.')

        #         # Finish if device reports not running anymore
        #         if status == self.Status.NOT_RUNNING:
        #             logging.info('Iontophoresis finished.')
        #             break

        #         # Enforce duration ceiling
        #         if asyncio.get_running_loop().time() >= deadline:
        #             logging.info('Iontophoresis duration elapsed -- stopping.')
        #             await self.device.write_ctrl_command(self.Command.STOP)
        #             # Briefly wait for device to acknowledge stop
        #             for _ in range(10):
        #                 if await self.is_done():
        #                     break
        #                 await asyncio.sleep(poll_interval)
        #             break

        #         await asyncio.sleep(poll_interval)
        # finally:
        #     # Ensure technique is stopped even on exceptions/cancellations
        #     try:
        #         await self.device.write_ctrl_command(self.Command.STOP)
        #     except Exception as e:
        #         logging.debug('STOP may already be in effect: %s', e)

        # No streamed data for this technique, but keep the return shape consistent.
        return np.empty((0, 2), dtype=float)


    def notification_handler(self, _: int, data: bytes) -> None:
        """
        No notificaitons for this technique
        """
        pass
