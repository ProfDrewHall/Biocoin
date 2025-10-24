#######################################################################################################################
#
# BioCoin Device Interface
#
# Manages BLE connection and GATT-level interactions with the BioCoin hardware. This module provides methods to
# connect, configure, and communicate with the device. It abstracts BLE services such as battery level, control
# commands, and technique configuration.
#
# Written By:
#   - Drew Hall (DrewHall@ucsd.edu)
#   - Risab Sankar (rsankar@ucsd.edu)
#   - Tyler Hack (thack@ucsd.edu)
#
# Revision History:
#   - 28 June 2025: Initial creation and refactor to support multiple techniques
#
# Known Issues:
#
#######################################################################################################################
import asyncio
import logging
from collections.abc import Callable

from bleak import BleakClient

from biocoin.utils.ble_util import find_address_by_uuid, list_all_devices

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

logger = logging.getLogger(__name__)


class BioCoinDevice:
    """
    Manages BLE communication with the BioCoin device, including connection, configuration, and basic command utilities
    """
    def __init__(self, uuid: str | None = None):
        """
        Initialize the BioCoinDevice instance.

        Parameters:
            - uuid (str | None): Optional UUID to search for the device. Defaults to service UUID.
        """
        self.uuid = uuid or UUID_DEVICE_SERVICE
        self.client: BleakClient | None = None

    async def connect(self, name: str | None = None, max_retries=5, timeout=15) -> None:
        """
        Connect to the BioCoin device using BLE and store the client reference.

        Parameters:
            - name (str | None): Optional name to search for. Defaults to "Biocoin".

        Raises:
            - RuntimeError: If the device cannot be found or connection fails.
        """
        name = name or DEVICE_NAME
        logging.info(f'Searching for the BioCoin device named "{name}"...')
        device_name, address = await find_address_by_uuid(self.uuid, name=name)

        if not address:
            # Try printing available devices to help the user debug
            logging.error(f'BioCoin device named "{name}" not found. Listing available BLE devices:\n')

            devices = await list_all_devices()
            for name, address, uuids in devices:
                logging.info(f'  - {name} @ {address}, Services: {uuids}')

            raise RuntimeError(f'BioCoin device named "{name}" not found. Ensure it is powered on and in range.')

        logging.info(f'Found {device_name} device at {address}!')

        self.client = BleakClient(address, timeout=timeout)
        delay = 0.5  # Initial delay before retrying connection

        for attempt in range(1, max_retries + 1):
            try:
                # use_services_cache=False can fix bad cached DB on macOS/Linux
                await self.client.connect(timeout=timeout, use_services_cache=False)
                if not self.client.is_connected:
                    raise RuntimeError('Connected returned but client.is_connected is False')

                # Trigger service resolution sanity check
                _ = self.client.services  # populated after connect in modern Bleak
                logging.info(f'Connected to BioCoin device at {address}.')
                return

            except asyncio.TimeoutError:
                logging.warning(f'Connect attempt {attempt}/{max_retries} failed due to timeout. Retrying...')
                # Helpful recovery steps between retries
                await asyncio.sleep(delay)
                delay = min(delay * 2, 4.0)

        raise RuntimeError(f'Failed to connect to the BioCoin device at {address}.')

    async def disconnect(self) -> None:
        """
        Disconnect from the BioCoin device if connected.
        """
        if self.client and self.client.is_connected:
            await self.client.disconnect()
            logging.info('Disconnected from BioCoin device.')

    async def get_battery_level(self) -> int:
        """
        Read and return the battery level from the device.

        Returns:
            - int: Battery level percentage

        Raises:
            - Exception: If reading the battery level fails.
        """
        try:
            battery_level = await self.client.read_gatt_char(BATTERY_LEVEL_UUID)
            level = int(battery_level[0])
            logging.info(f'Battery Level: {level}%')
            return level
        except Exception as e:
            logging.exception(f'Failed to retrieve battery level: {e}')
            raise

    async def change_device_name(self, name: str) -> None:
        """
        Change the device name. Note, this will reboot the device and the connection will be lost. Must be handled
        correctly. Currently it does nothing. Improved version would reconnect.

        Parameters:
            - name (str): New name for the device. Must be less than 240 characters.

        """
        name_bytes = bytearray(name, 'utf-8')
        if len(name_bytes) >= 240:
            raise ValueError('Device name must be less than 240 characters.')

        await self.client.write_gatt_char(BIOCOIN_UUID_CHR_NAME, name_bytes)

    def get_characteristic(self, uuid: str):
        """
        Retrieve a GATT characteristic by UUID.

        Parameters:
            - uuid (str): UUID of the desired characteristic

        Returns:
            - characteristic: BLE GATT characteristic

        Raises:
            - Exception: If the characteristic is not found.
        """
        char = self.client.services.get_characteristic(uuid)
        if not char:
            raise Exception(f'Characteristic {uuid} not found.')
        return char

    async def start_notify(self, char_uuid: str, handler: Callable[[int, bytes], None]) -> None:
        """
        Start receiving BLE notifications from the specified characteristic.

        Parameters:
            - char_uuid (str): UUID of the characteristic to subscribe to
            - handler (Callable): Callback function to handle received notifications
        """
        await self.client.start_notify(char_uuid, handler)

    async def stop_notify(self, char_uuid: str) -> None:
        """
        Stop receiving BLE notifications from the specified characteristic.

        Parameters:
            - char_uuid (str): UUID of the characteristic to unsubscribe from
        """
        await self.client.stop_notify(char_uuid)

    async def write_ctrl_command(self, command_byte: int) -> None:
        """
        Write a control command byte to the control characteristic.

        Parameters:
            - command_byte (int): Byte value representing a control command (e.g., 0x01 to start)
        """
        await self.client.write_gatt_char(BIOCOIN_UUID_CHR_ECHEMCTRL, bytearray([command_byte]))

    async def write_technique_config(self, config_bytes: bytes) -> None:
        """
        Write configuration data for a measurement technique.

        Parameters:
            - config_bytes (bytes): Packed configuration payload to send to the device
        """
        tech_char = self.get_characteristic(BIOCOIN_UUID_CHR_ECHEMTECH)
        await self.client.write_gatt_char(tech_char, bytearray(config_bytes))
