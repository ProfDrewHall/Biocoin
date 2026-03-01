"""
BLE device interface for Biocoin hardware.
"""

import asyncio
from collections.abc import Awaitable, Callable
from typing import Any

from bleak import BleakClient

from biocoin.utils.ble_util import find_address_by_uuid, list_all_devices
from utils.logging_util import get_logger

# UUIDs
UUID_DEVICE_SERVICE: str = '00001523-1212-efde-1523-785feabc93aa'
BIOCOIN_UUID_CHR_ECHEMTECH: str = '0000152a-1212-efde-1523-785feabc93aa'
BIOCOIN_UUID_CHR_ECHEMCTRL: str = '00001528-1212-efde-1523-785feabc93aa'
BIOCOIN_UUID_CHR_ECHEMDATA: str = '00001529-1212-efde-1523-785feabc93aa'
BATTERY_SERVICE_UUID: str = '0000180F-0000-1000-8000-00805f9b34fb'
BATTERY_LEVEL_UUID: str = '00002A19-0000-1000-8000-00805f9b34fb'
BIOCOIN_UUID_CHR_STATUS: str = '00001524-1212-efde-1523-785feabc93aa'
BIOCOIN_UUID_CHR_NAME: str = '00001525-1212-efde-1523-785feabc93aa'

DEVICE_NAME: str = 'Biocoin'
DEFAULT_CONNECT_MAX_RETRIES: int = 5
DEFAULT_CONNECT_TIMEOUT_S: float = 15.0
INITIAL_RETRY_DELAY_S: float = 0.5
MAX_RETRY_DELAY_S: float = 4.0

logger = get_logger(__name__)


class BiocoinDevice:
    """
    Manages BLE communication with the Biocoin device, including connection, configuration, and basic command utilities.

    """
    def __init__(self, uuid: str | None = None):
        """
        Initialize the BiocoinDevice instance.

        Parameters:
            - uuid (str | None): Optional UUID to search for the device. Defaults to service UUID.
        Returns:
            - None
        """
        self.uuid = uuid or UUID_DEVICE_SERVICE
        self.client: BleakClient | None = None

    def _require_client(self, *, connected: bool = True) -> BleakClient:
        """
        Return the active BLE client or raise a clear connection error.

        Parameters:
            - connected (bool): Require an active connected BLE client when ``True``.
        Returns:
            - BleakClient: Active BLE client instance.
        """
        if self.client is None:
            raise RuntimeError('BLE client not initialized. Call connect() first.')
        if connected and not getattr(self.client, 'is_connected', True):
            raise RuntimeError('BLE client not connected.')
        return self.client

    async def connect(
        self,
        name: str | None = None,
        max_retries: int = DEFAULT_CONNECT_MAX_RETRIES,
        timeout: float = DEFAULT_CONNECT_TIMEOUT_S,
    ) -> None:
        """
        Connect to the Biocoin device using BLE and store the client reference.

        Parameters:
            - name (str | None): Optional name to search for. Defaults to "Biocoin".
            - max_retries (int): Maximum number of connection attempts.
            - timeout (float): Per-attempt timeout in seconds.
        Returns:
            - None

        Raises:
            - RuntimeError: If the device cannot be found or connection fails.
        """
        requested_name = name or DEVICE_NAME
        logger.info(f'Searching for the Biocoin device named "{requested_name}"...')
        device_name, address = await find_address_by_uuid(self.uuid, name=requested_name)

        if not address:
            # Try printing available devices to help the user debug
            logger.error(f'Biocoin device named "{requested_name}" not found. Listing available BLE devices:')

            devices = await list_all_devices()
            for discovered_name, discovered_address, uuids in devices:
                logger.info(f'  - {discovered_name} @ {discovered_address}, Services: {uuids}')

            raise RuntimeError(
                f'Biocoin device named "{requested_name}" not found. Ensure it is powered on and in range.'
            )

        logger.info(f'Found {device_name} device at {address}!')

        self.client = BleakClient(address, timeout=timeout)
        delay = INITIAL_RETRY_DELAY_S

        for attempt in range(1, max_retries + 1):
            try:
                # use_services_cache=False can fix bad cached DB on macOS/Linux
                await self.client.connect(timeout=timeout, use_services_cache=False)
                if not self.client.is_connected:
                    raise RuntimeError('Connected returned but client.is_connected is False')

                # Trigger service resolution sanity check
                _ = self.client.services  # populated after connect in modern Bleak
                logger.info(f'Connected to Biocoin device at {address}.')
                return

            except TimeoutError:
                logger.warning(f'Connect attempt {attempt}/{max_retries} failed due to timeout. Retrying...')
                # Helpful recovery steps between retries
                await asyncio.sleep(delay)
                delay = min(delay * 2, MAX_RETRY_DELAY_S)

        raise RuntimeError(f'Failed to connect to the Biocoin device at {address}.')

    async def disconnect(self) -> None:
        """
        Disconnect from the Biocoin device if connected.

        Parameters:
            - None
        Returns:
            - None
        """
        if self.client and self.client.is_connected:
            await self.client.disconnect()
            logger.info('Disconnected from Biocoin device.')

    async def get_battery_level(self) -> int:
        """
        Read and return the battery level from the device.

        Parameters:
            - None
        Returns:
            - int: Battery level percentage

        Raises:
            - Exception: If reading the battery level fails.
        """
        client = self._require_client()
        try:
            battery_level = await client.read_gatt_char(BATTERY_LEVEL_UUID)
            level = int(battery_level[0])
            logger.debug(f'Battery Level: {level}%')
            return level
        except Exception:
            logger.exception('Failed to retrieve battery level.')
            raise

    async def change_device_name(self, name: str) -> None:
        """
        Change the device name. Note, this will reboot the device and the connection will be lost. Must be handled
        correctly. Currently it does nothing. Improved version would reconnect.

        Parameters:
            - name (str): New name for the device. Must be less than 240 characters.
        Returns:
            - None
        """
        name_bytes = bytearray(name, 'utf-8')
        if len(name_bytes) >= 240:
            raise ValueError('Device name must be less than 240 characters.')

        client = self._require_client()
        await client.write_gatt_char(BIOCOIN_UUID_CHR_NAME, name_bytes)

    def get_characteristic(self, uuid: str) -> Any:
        """
        Retrieve a GATT characteristic by UUID.

        Parameters:
            - uuid (str): UUID of the desired characteristic

        Returns:
            - Any: BLE GATT characteristic

        Raises:
            - LookupError: If the characteristic is not found.
        """
        client = self._require_client()
        char = client.services.get_characteristic(uuid)
        if not char:
            raise LookupError(f'Characteristic {uuid} not found.')
        return char

    async def start_notify(
        self,
        char_uuid: str,
        handler: Callable[[int, bytes], Awaitable[None] | None],
    ) -> None:
        """
        Start receiving BLE notifications from the specified characteristic.

        Parameters:
            - char_uuid (str): UUID of the characteristic to subscribe to
            - handler (Callable): Callback function to handle received notifications
        Returns:
            - None
        """
        client = self._require_client()
        await client.start_notify(char_uuid, handler)  # type: ignore[arg-type]

    async def stop_notify(self, char_uuid: str) -> None:
        """
        Stop receiving BLE notifications from the specified characteristic.

        Parameters:
            - char_uuid (str): UUID of the characteristic to unsubscribe from
        Returns:
            - None
        """
        client = self._require_client()
        await client.stop_notify(char_uuid)

    async def write_ctrl_command(self, command_byte: int) -> None:
        """
        Write a control command byte to the control characteristic.

        Parameters:
            - command_byte (int): Byte value representing a control command (e.g., 0x01 to start)
        Returns:
            - None
        """
        client = self._require_client()
        await client.write_gatt_char(BIOCOIN_UUID_CHR_ECHEMCTRL, bytearray([command_byte]))

    async def write_technique_config(self, config_bytes: bytes) -> None:
        """
        Write configuration data for a measurement technique.

        Parameters:
            - config_bytes (bytes): Packed configuration payload to send to the device
        Returns:
            - None
        """
        client = self._require_client()
        tech_char = self.get_characteristic(BIOCOIN_UUID_CHR_ECHEMTECH)
        await client.write_gatt_char(tech_char, bytearray(config_bytes))


