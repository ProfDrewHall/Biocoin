########################################################################################################################
#
# BioCoin BLE Utility Functions
#
# Provides helper functions for scanning and identifying BLE devices compatible with the BioCoin platform.
# Currently supports discovery by advertised service UUID and optional device name filtering.
#
# Written By:
#   - Drew Hall (drewhall@ucsd.edu)
#
# Known Issues:
#   - Does not support reconnection handling
#   - Assumes BLE advertising packets always include service UUIDs
#
# Revision History:
#   - 28 June 2025: Initial implementation of BLE discovery utilities
#
########################################################################################################################
import logging

from bleak import BleakClient, BleakScanner

# Setup logging
logger = logging.getLogger(__name__)


async def list_ble_services_and_characteristics(client: BleakClient) -> None:
    """
    Lists all services and their characteristics from a connected BLE device

    Parameters:
        client (BleakClient): An active, connected BleakClient instance

    Returns:

    """
    logger.info('Listing available services and characteristics...')

    if not client.is_connected:
        raise RuntimeError('Client is not connected. Please connect to a device first.')

    # Get all services from the client
    await client.get_services()

    logging.info('Available services and characteristics:')
    for service in client.services:
        logging.info(f'Service: {service.uuid}')
        for char in service.characteristics:
            logging.info(f'  Characteristic: {char.uuid} - Properties: {char.properties}')


async def list_all_devices() -> list[tuple[str, str, list[str]]]:
    """
    List all nearby BLE devices

    Returns:
        List of tuples: (device name, device address, list of advertised service UUIDs)
    """
    logging.info('Scanning for devices...')
    devices = await BleakScanner.discover(return_adv=True)
    results = []

    for device, adv_data in devices.values():
        name = device.name or 'Unknown'
        address = device.address
        uuids = adv_data.service_uuids or []
        logging.debug(f'\tDevice: {name} ({address}) | UUIDs: {uuids}')
        results.append((name, address, uuids))

    return results


async def find_address_by_uuid(uuid: str, name: str | None = None) -> tuple[str, str] | tuple[None, None]:
    """
    Find the address of the device by matching the advertised service UUID,
    and optionally filtering by device name.

    Parameters:
        uuid (str): UUID of the device service to match
        name (Optional[str]): Optional name of the device to match

    Returns:
        Tuple[str, str] or (None, None): (device name, device address), or (None, None) if not found
    """
    devices = await list_all_devices()

    for device_name, address, uuids in devices:
        if uuid in uuids:
            if name is None or (device_name and name.lower() in device_name.lower()):
                return device_name, address

    return None, None
