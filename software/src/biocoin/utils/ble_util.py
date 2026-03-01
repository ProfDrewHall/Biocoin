"""
BLE utility helpers for discovery and advertised-service filtering.
"""

from bleak import BleakClient, BleakScanner

from utils.logging_util import get_logger

logger = get_logger(__name__)


async def list_ble_services_and_characteristics(client: BleakClient) -> None:
    """
    Lists all services and their characteristics from a connected BLE device.

    Parameters:
        - client (BleakClient): An active, connected BleakClient instance.
    Returns:
        - None
    """
    logger.info('Listing available services and characteristics...')

    if not client.is_connected:
        raise RuntimeError('Client is not connected. Please connect to a device first.')

    # Get all services from the client
    await client.get_services()

    logger.info('Available services and characteristics:')
    for service in client.services:
        logger.info(f'Service: {service.uuid}')
        for char in service.characteristics:
            logger.info(f'  Characteristic: {char.uuid} - Properties: {char.properties}')


async def list_all_devices() -> list[tuple[str, str, list[str]]]:
    """
    List all nearby BLE devices.

    Parameters:
        - None
    Returns:
        - list[tuple[str, str, list[str]]]: Tuples of (device name, device address, advertised UUIDs).
    """
    logger.info('Scanning for devices...')
    devices = await BleakScanner.discover(return_adv=True)
    results = []

    for device, adv_data in devices.values():
        name = device.name or 'Unknown'
        address = device.address
        uuids = adv_data.service_uuids or []
        logger.debug(f'\tDevice: {name} ({address}) | UUIDs: {uuids}')
        results.append((name, address, uuids))

    return results


async def find_address_by_uuid(uuid: str, name: str | None = None) -> tuple[str | None, str | None]:
    """
    Find the address of the device by matching the advertised service UUID,
    and optionally filtering by device name.

    Parameters:
        - uuid (str): UUID of the device service to match.
        - name (str | None): Optional name of the device to match.

    Returns:
        - tuple[str | None, str | None]: (device name, device address), or (None, None) if not found.
    """
    devices = await list_all_devices()

    for device_name, address, uuids in devices:
        if uuid in uuids:
            if name is None or (device_name and name.lower() in device_name.lower()):
                return device_name, address

    return None, None


