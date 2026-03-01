"""
Shared base class for Biocoin electrochemical techniques.
"""

import asyncio
import struct
from abc import ABC, abstractmethod
from collections.abc import Callable
from enum import IntEnum

import numpy as np

from biocoin.device import BIOCOIN_UUID_CHR_ECHEMDATA, BIOCOIN_UUID_CHR_STATUS, BiocoinDevice
from utils.logging_util import get_logger

logger = get_logger(__name__)

class BaseTechnique[QueueItemT](ABC):
    """
    Abstract base class for all Biocoin electrochemical techniques. Manages notification registration, data buffering,
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
        CA = 0x01
        CV = 0x02
        DPV = 0x03
        IMP = 0x04
        OCP = 0x05
        SWV = 0x06
        TEMP = 0x10
        IONTOPHORESIS = 0x20

    def __init__(self, device: BiocoinDevice, char_uuid: str | None = None):
        """
        Initialize the base technique with a reference to the BLE device.

        Parameters:
            - device (BiocoinDevice): The connected Biocoin device
            - char_uuid (str): UUID of the characteristic that sends data for this technique
        Returns:
            - None
        """
        self.device = device
        self.char_uuid = char_uuid or BIOCOIN_UUID_CHR_ECHEMDATA
        self.byte_buffer = bytearray()
        self.data_queue: asyncio.Queue[QueueItemT] = asyncio.Queue()

    def _drain_queue(self) -> list[QueueItemT]:
        """
        Drain and return all currently queued items without blocking.

        Parameters:
            - None
        Returns:
            - list[QueueItemT]: Drained queue items.
        """
        items: list[QueueItemT] = []
        while True:
            try:
                items.append(self.data_queue.get_nowait())
            except asyncio.QueueEmpty:
                break
        return items

    def _ingest_float32_samples(self: 'BaseTechnique[float]', data: bytes, *, label: str | None = None) -> None:
        """
        Parse little-endian float32 samples from notification data into ``data_queue``.

        Parameters:
            - data (bytes): Raw notification bytes.
            - label (str | None): Optional label for debug logs.
        Returns:
            - None
        """
        self.byte_buffer.extend(data)
        logger.debug(f'Received: {data!r}')

        while len(self.byte_buffer) >= 4:
            chunk = self.byte_buffer[:4]
            del self.byte_buffer[:4]
            value = struct.unpack('<f', chunk)[0]
            self.data_queue.put_nowait(value)
            if label is not None:
                logger.debug(f'Received {label} value: {value}')

    def _ingest_float32_pairs(
        self: 'BaseTechnique[tuple[float, float]]',
        data: bytes,
        *,
        label: str | None = None,
        second_transform: Callable[[float], float] | None = None,
    ) -> None:
        """
        Parse little-endian float32 pairs from notification data into ``data_queue``.

        Parameters:
            - data (bytes): Raw notification bytes.
            - label (str | None): Optional label for debug logs.
            - second_transform (Callable[[float], float] | None): Optional transform for second value.
        Returns:
            - None
        """
        self.byte_buffer.extend(data)
        logger.debug(f'Received: {data!r}')

        while len(self.byte_buffer) >= 8:
            chunk = self.byte_buffer[:8]
            del self.byte_buffer[:8]
            first, second = struct.unpack('<ff', chunk)
            if second_transform is not None:
                second = second_transform(second)
            self.data_queue.put_nowait((first, second))
            if label is not None:
                logger.debug(f'Received {label} value: ({first}, {second})')

    async def _run_fixed_duration(self, duration: float, *, flush_delay: float = 1.0) -> list[QueueItemT]:
        """
        Run a notification-based technique for a fixed host-controlled duration.

        Parameters:
            - duration (float): Host-side run duration in seconds.
            - flush_delay (float): Extra post-stop drain delay in seconds.
        Returns:
            - list[QueueItemT]: Drained samples collected during the run.
        """
        if duration < 0:
            raise ValueError('duration must be >= 0')
        if flush_delay < 0:
            raise ValueError('flush_delay must be >= 0')

        await self.start()
        try:
            await self.device.write_ctrl_command(self.Command.START)
            await asyncio.sleep(duration)
            await self.device.write_ctrl_command(self.Command.STOP)
            if flush_delay > 0:
                await asyncio.sleep(flush_delay)
        finally:
            await self.stop()

        return self._drain_queue()

    async def _run_streaming_until_done(
        self,
        duration: float,
        *,
        max_polls: int = 10,
        poll_interval_cap: float = 2.0,
    ) -> list[QueueItemT]:
        """
        Run a notification-based technique and poll device status for completion.

        Parameters:
            - duration (float): Expected run duration in seconds.
            - max_polls (int): Number of completion polls after duration.
            - poll_interval_cap (float): Maximum poll interval in seconds.
        Returns:
            - list[QueueItemT]: Drained samples collected during the run.
        """
        if duration < 0:
            raise ValueError('duration must be >= 0')
        if max_polls <= 0:
            raise ValueError('max_polls must be > 0')
        if poll_interval_cap <= 0:
            raise ValueError('poll_interval_cap must be > 0')

        poll_interval = min(max(0.1 * duration, 0.01), poll_interval_cap)

        await self.start()
        try:
            await self.device.write_ctrl_command(self.Command.START)
            await asyncio.sleep(duration)

            for _ in range(max_polls):
                if await self.is_done():
                    break
                await asyncio.sleep(poll_interval)
            else:
                await self.device.write_ctrl_command(self.Command.STOP)
                total_wait = duration + max_polls * poll_interval
                raise TimeoutError(f'Device did not finish after {total_wait:.3f}s')
        finally:
            await self.stop()

        return self._drain_queue()

    async def start(self) -> None:
        """
        Begin BLE notifications and prepare for data reception.

        Parameters:
            - None
        Returns:
            - None
        """
        await self.clear_queue()
        try:
            await self.device.start_notify(self.char_uuid, self.notification_handler)
            logger.info('Notification handler registered successfully.')
        except Exception:
            logger.exception('Failed to register notification handler.')
            raise

    async def stop(self) -> None:
        """
        Stop BLE notifications and finalize data reception.

        Parameters:
            - None
        Returns:
            - None
        """
        await self.device.stop_notify(self.char_uuid)

    async def clear_queue(self) -> None:
        """
        Clear any existing data in the queue.

        Parameters:
            - None
        Returns:
            - None
        """
        while not self.data_queue.empty():
            try:
                self.data_queue.get_nowait()
            except asyncio.QueueEmpty:
                break

    async def get_status(self) -> 'BaseTechnique.Status':
        """
        Read the device status characteristic and return a typed Status enum.

        Parameters:
            - None
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

        logger.debug(f'Device status read: {status.name} ({int(status)})')
        return status

    def is_fault_status(self, status: 'BaseTechnique.Status') -> bool:
        """
        Return True if the given status indicates a fault condition.

        Parameters:
            - status (BaseTechnique.Status): Status value to evaluate.
        Returns:
            - bool: ``True`` when the status indicates a fault.
        """
        return status in {
            self.Status.INVALID_PARAMETERS,
            self.Status.ERROR,
            self.Status.CURRENT_LIMIT_EXCEEDED}

    async def is_running(self) -> bool:
        """
        Convenience wrapper to check if the technique is currently running.

        Parameters:
            - None
        Returns:
            - bool: ``True`` when device status is RUNNING.
        """
        return (await self.get_status()) == self.Status.RUNNING

    async def is_done(self) -> bool:
        """
        Convenience wrapper to check if the technique has finished (i.e., not RUNNING).

        Parameters:
            - None
        Returns:
            - bool: ``True`` when device status is not RUNNING.
        """
        return (await self.get_status()) != self.Status.RUNNING

    async def assert_config_ok(self) -> None:
        """
        After sending the configuration, confirm device accepted parameters.

        Parameters:
            - None
        Returns:
            - None
        """
        await asyncio.sleep(1)  # Give firmware a tick to update status if needed.
        status = await self.get_status()
        if status == self.Status.INVALID_PARAMETERS:
            raise ValueError('Invalid parameters')
        if status == self.Status.ERROR:
            raise RuntimeError('Device reported ERROR after configuration')
        if status == self.Status.CURRENT_LIMIT_EXCEEDED:
            raise RuntimeError('Device reported CURRENT_LIMIT_EXCEEDED after configuration')

    def _segment(self, a: float, b: float, step: float) -> np.ndarray:
        """
        Build a->b with full steps of 'step' and a final partial step to land exactly on b

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

        Parameters:
            - **kwargs: Technique-specific configuration values.
        Returns:
            - None
        """
        pass

    @abstractmethod
    async def run(self, **kwargs) -> np.ndarray:
        """
        Execute the measurement and return processed results.

        Parameters:
            - **kwargs: Technique-specific runtime options.
        Returns:
            - np.ndarray: Technique result array.
        """
        pass

    @abstractmethod
    async def notification_handler(self, sender: int, data: bytes) -> None:
        """
        Parse incoming BLE data. To be implemented by each technique.

        Parameters:
            - sender (int): BLE sender identifier.
            - data (bytes): Raw notification bytes.
        Returns:
            - None
        """
        pass


