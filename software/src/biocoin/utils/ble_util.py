"""
BLE utility helpers for discovery and advertised-service filtering.
"""

from bleak import BleakClient, BleakScanner

from utils.logging_util import get_logger

logger = get_logger(__name__)
DEFAULT_SERVICE_PROBE_TIMEOUT_S = 5.0


def _name_matches(device_name: str, expected_name: str | None) -> bool:
    """
    Return whether a discovered device name matches the optional filter.

    Parameters:
        - device_name (str): Name reported by the BLE scan.
        - expected_name (str | None): Optional case-insensitive name substring.
    Returns:
        - bool: ``True`` when the filter is unset or the name matches it.
    """
    return expected_name is None or expected_name.lower() in device_name.lower()


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


async def device_has_service(
    address: str,
    service_uuid: str,
    *,
    timeout: float = DEFAULT_SERVICE_PROBE_TIMEOUT_S,
) -> bool:
    """
    Connect to a BLE device and verify that it exposes a specific GATT service.

    Parameters:
        - address (str): BLE device address.
        - service_uuid (str): UUID of the service to look for.
        - timeout (float): Connection timeout in seconds.
    Returns:
        - bool: ``True`` when the service is present after a successful connection.
    """
    normalized_uuid = service_uuid.lower()
    client = BleakClient(address, timeout=timeout)

    try:
        await client.connect(timeout=timeout, use_services_cache=False)
        services = client.services
        if not services:
            await client.get_services()
            services = client.services

        return any(service.uuid.lower() == normalized_uuid for service in services)
    except Exception:
        logger.debug(f'Unable to verify service {service_uuid} on {address}.', exc_info=True)
        return False
    finally:
        if client.is_connected:
            try:
                await client.disconnect()
            except Exception:
                logger.debug(f'Failed to disconnect cleanly from {address} after service probe.', exc_info=True)


async def list_connectable_devices_with_service(
    service_uuid: str,
    *,
    name: str | None = None,
    timeout: float = DEFAULT_SERVICE_PROBE_TIMEOUT_S,
) -> list[tuple[str, str, list[str]]]:
    """
    List nearby BLE devices that can be connected to and expose the target service.

    Parameters:
        - service_uuid (str): UUID of the required GATT service.
        - name (str | None): Optional case-insensitive name filter.
        - timeout (float): Per-device connection timeout in seconds.
    Returns:
        - list[tuple[str, str, list[str]]]: Tuples of (device name, device address, advertised UUIDs).
    """
    normalized_uuid = service_uuid.lower()
    devices = await list_all_devices()
    connectable_devices = []
    probed_addresses: set[str] = set()

    for device_name, address, uuids in devices:
        if not _name_matches(device_name, name):
            continue

        if any(uuid.lower() == normalized_uuid for uuid in uuids):
            logger.debug(f'Keeping {device_name} ({address}) based on advertised service {service_uuid}.')
            connectable_devices.append((device_name, address, uuids))
            probed_addresses.add(address)

    for device_name, address, uuids in devices:
        if address in probed_addresses or not _name_matches(device_name, name):
            continue

        # Some devices omit service UUIDs from advertisements, so probe only likely matches.
        logger.debug(f'Probing {device_name} ({address}) for service {service_uuid}...')
        if await device_has_service(address, service_uuid, timeout=timeout):
            connectable_devices.append((device_name, address, uuids))

    return connectable_devices


async def find_address_by_uuid(uuid: str, name: str | None = None) -> tuple[str | None, str | None]:
    """
    Find the address of a connectable device exposing the target service UUID,
    and optionally filter by device name.

    Parameters:
        - uuid (str): UUID of the device service to match.
        - name (str | None): Optional name of the device to match.

    Returns:
        - tuple[str | None, str | None]: (device name, device address), or (None, None) if not found.
    """
    devices = await list_connectable_devices_with_service(uuid, name=name)

    for device_name, address, uuids in devices:
        return device_name, address

    return None, None
