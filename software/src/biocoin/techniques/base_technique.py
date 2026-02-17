########################################################################################################################
#
# BioCoin Base Technique Class
#
# Abstract base class for implementing electrochemical measurement techniques.
# Provides shared functionality such as notification management, data queueing, and BLE interaction patterns.
# Each specific technique subclass (e.g., CA, DPV, EIS) is responsible for implementing configuration, execution,
# and BLE notification parsing.
#
# Written By:
#   - Drew Hall (DrewHall@ucsd.edu)
#   - Risab Sankar (rsankar@ucsd.edu)
#   - Tyler Hack (thack@ucsd.edu)
#
# Known Issues:
#   - Does not enforce timeout or data validity checks in base run() method
#
# Revision History:
#   - 28 June 2025: Initial creation for BioCoin device abstraction layer
#
########################################################################################################################
import asyncio
import logging
from abc import ABC, abstractmethod
from enum import IntEnum
import numpy as np


from biocoin.device import BIOCOIN_UUID_CHR_ECHEMDATA, BIOCOIN_UUID_CHR_STATUS, BioCoinDevice

logger = logging.getLogger(__name__)


class BaseTechnique(ABC):
    """
    Abstract base class for all BioCoin electrochemical techniques. Manages notification registration, data buffering,
    and basic lifecycle control.
    """

    # Commands to control the technique
    class Command(IntEnum):
        START = 0x01
        STOP = 0xFF

    # Status code
    class Status(IntEnum):
        NOT_RUNNING = 0x00
        INVALID_PARAMETERS = 0x01
        RUNNING = 0x02
        ERROR = 0x03
        CURRENT_LIMIT_EXCEEDED = 0x04
  
    # Technique ID definitions
    class Technique(IntEnum):
        CA: int = 0x01
        CV: int = 0x02
        DPV: int = 0x03
        IMP: int = 0x04
        OCP: int = 0x05
        SWV: int = 0x06
        TEMP: int = 0x10
        IONTOPHORESIS: int = 0x20

    def __init__(self, device: BioCoinDevice, char_uuid: str | None = None):
        """
        Initialize the base technique with a reference to the BLE device

        Parameters:
            - device (BioCoinDevice): The connected BioCoin device
            - char_uuid (str): UUID of the characteristic that sends data for this technique
        """
        self.device = device
        self.char_uuid = char_uuid or BIOCOIN_UUID_CHR_ECHEMDATA
        self.byte_buffer = bytearray()
        self.data_queue: asyncio.Queue = asyncio.Queue()

    async def start(self) -> None:
        """
        Begin BLE notifications and prepare for data reception
        """
        await self.clear_queue()
        try:
            await self.device.start_notify(self.char_uuid, self.notification_handler)
            logging.info('Notification handler registered successfully.')
        except Exception as e:
            logging.error(f'Failed to register notification handler: {e}')

    async def stop(self) -> None:
        """
        Stop BLE notifications and finalize data reception
        """
        await self.device.stop_notify(self.char_uuid)

    async def clear_queue(self) -> None:
        """
        Clear any existing data in the queue
        """
        while not self.data_queue.empty():
            try:
                self.data_queue.get_nowait()
                self.data_queue.task_done()
            except asyncio.QueueEmpty:
                break

    async def get_status(self) -> 'BaseTechnique.Status':
        """
        Read the device status characteristic and return a typed Status enum

        Returns:
            - BaseTechnique.Status: Current device/technique status

        Raises:
            - RuntimeError: If the BLE client is not connected
            - ValueError: If the returned status byte is not a valid Status
        """
        if self.device.client is None or not self.device.client.is_connected:
            raise RuntimeError('BLE client not connected')

        raw = await self.device.client.read_gatt_char(BIOCOIN_UUID_CHR_STATUS)
        if not raw:
            raise ValueError('Empty status response from device')

        try:
            status = self.Status(int(raw[0]))
        except Exception as e:
            raise ValueError(f'Unknown status: {raw!r}') from e

        logger.debug('Device status read: %s (%d)', status.name, int(status))
        return status

    def is_fault_status(self, status: 'BaseTechnique.Status') -> bool:
        """
        Return True if the given status indicates a fault condition
        """
        return status in {
            self.Status.INVALID_PARAMETERS,
            self.Status.ERROR,
            self.Status.CURRENT_LIMIT_EXCEEDED}

    async def is_running(self) -> bool:
        """
        Convenience wrapper to check if the technique is currently running.
        """
        return (await self.get_status()) == self.Status.RUNNING

    async def is_done(self) -> bool:
        """
        Convenience wrapper to check if the technique has finished (i.e., not RUNNING).
        """
        return (await self.get_status()) != self.Status.RUNNING
  
    async def assert_config_ok(self) -> None:
        """
        After sending the configuration, confirm device accepted parameters.

        Parameters:

        Exceptions:
            - ValueError (if parameters invalid)

        Raises ValueError on INVALID_PARAMETERS, RuntimeError on other non-OK states.
        """
        await asyncio.sleep(1)                      # Give firmware a tick to update status if needed
        status = await self.get_status()
        if status == self.Status.INVALID_PARAMETERS:
            raise ValueError('Invalid parameters')

    def _segment(self, a: float, b: float, step: float) -> np.ndarray:
        """
        Build a→b with full steps of 'step' and a final partial step to land exactly on b

        Parameters:
            - a (float): Start value
            - b (float): End value
            - step (float): Step size, must be > 0

        Returns:
            - np.ndarray: Array of values from a to b with specified step size
        """
        step = float(abs(step))
        if step == 0:
            raise ValueError('step must be nonzero')
        if a == b:
            return np.array([float(a)], dtype=float)

        sgn = 1.0 if b > a else -1.0
        dist = abs(b - a)

        # Minimum number of full steps needed to reach b (without overshooting)
        num_steps = int(np.round(dist / step))

        # Indices 0..n_steps inclusive gives n_steps+1 points including the start
        idx = np.arange(num_steps + 1, dtype=float)
        seg = a + sgn * step * idx
        return seg.astype(float, copy=False)

    @abstractmethod
    async def configure(self, **kwargs) -> None:
        """
        Configure the technique with parameters and send them to the device.
        """
        pass

    @abstractmethod
    async def run(self, **kwargs) -> list:
        """
        Execute the measurement and return processed results.
        """
        pass

    @abstractmethod
    async def notification_handler(self, sender: int, data: bytes) -> None:
        """
        Parse incoming BLE data. To be implemented by each technique.
        """
        pass
