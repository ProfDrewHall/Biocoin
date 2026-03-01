from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest

import biocoin.device as device_module
import biocoin.utils.ble_util as ble_util_module
from biocoin.device import BIOCOIN_UUID_CHR_ECHEMTECH, BiocoinDevice
from biocoin.utils.ble_util import find_address_by_uuid, list_all_devices


@pytest.mark.asyncio
async def test_list_all_devices_transforms_scanner_results(monkeypatch: pytest.MonkeyPatch) -> None:
    fake_discovery = {
        'd1': (SimpleNamespace(name='Biocoin A', address='AA:BB'), SimpleNamespace(service_uuids=['1234', '9999'])),
        'd2': (SimpleNamespace(name=None, address='CC:DD'), SimpleNamespace(service_uuids=None)),
    }
    monkeypatch.setattr(ble_util_module.BleakScanner, 'discover', AsyncMock(return_value=fake_discovery))

    devices = await list_all_devices()

    assert devices == [
        ('Biocoin A', 'AA:BB', ['1234', '9999']),
        ('Unknown', 'CC:DD', []),
    ]


@pytest.mark.asyncio
async def test_find_address_by_uuid_honors_name_filter(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        ble_util_module,
        'list_all_devices',
        AsyncMock(
            return_value=[
                ('Random Device', '11:22', ['abcd']),
                ('Biocoin Unit 1', '33:44', ['0000']),
                ('Biocoin Unit 2', '55:66', ['abcd']),
            ]
        ),
    )

    found_name, found_address = await find_address_by_uuid('abcd', name='unit 2')
    missing_name, missing_address = await find_address_by_uuid('ffff', name='unit 2')

    assert (found_name, found_address) == ('Biocoin Unit 2', '55:66')
    assert (missing_name, missing_address) == (None, None)


@pytest.mark.asyncio
async def test_connect_raises_if_device_not_found(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(device_module, 'find_address_by_uuid', AsyncMock(return_value=(None, None)))
    monkeypatch.setattr(device_module, 'list_all_devices', AsyncMock(return_value=[]))
    device = BiocoinDevice()

    with pytest.raises(RuntimeError, match='not found'):
        await device.connect(name='missing', max_retries=1, timeout=0.01)


@pytest.mark.asyncio
async def test_connect_success_creates_connected_client(monkeypatch: pytest.MonkeyPatch) -> None:
    class FakeBleakClient:
        def __init__(self, address: str, timeout: float) -> None:
            self.address = address
            self.timeout = timeout
            self.is_connected = False
            self.services = {'ok': True}

        async def connect(self, timeout: float, use_services_cache: bool) -> None:
            assert use_services_cache is False
            assert timeout == self.timeout
            self.is_connected = True

    monkeypatch.setattr(device_module, 'find_address_by_uuid', AsyncMock(return_value=('Biocoin', 'AA:BB:CC')))
    monkeypatch.setattr(device_module, 'BleakClient', FakeBleakClient)

    device = BiocoinDevice()
    await device.connect(max_retries=1, timeout=0.5)

    assert isinstance(device.client, FakeBleakClient)
    assert device.client.address == 'AA:BB:CC'
    assert device.client.is_connected is True


def test_get_characteristic_raises_when_missing() -> None:
    device = BiocoinDevice()
    device.client = SimpleNamespace(services=SimpleNamespace(get_characteristic=lambda _: None))

    with pytest.raises(LookupError, match='Characteristic .* not found'):
        device.get_characteristic(BIOCOIN_UUID_CHR_ECHEMTECH)


@pytest.mark.asyncio
async def test_write_technique_config_uses_resolved_characteristic() -> None:
    write_gatt_char = AsyncMock()
    client = SimpleNamespace(
        write_gatt_char=write_gatt_char,
        services=SimpleNamespace(get_characteristic=lambda _: 'FAKE_CHAR'),
    )
    device = BiocoinDevice()
    device.client = client

    await device.write_technique_config(b'\x01\x02\x03')

    write_gatt_char.assert_awaited_once_with('FAKE_CHAR', bytearray(b'\x01\x02\x03'))


@pytest.mark.asyncio
async def test_disconnect_only_when_connected() -> None:
    connected_client = SimpleNamespace(is_connected=True, disconnect=AsyncMock())
    disconnected_client = SimpleNamespace(is_connected=False, disconnect=AsyncMock())

    device = BiocoinDevice()
    device.client = connected_client
    await device.disconnect()
    connected_client.disconnect.assert_awaited_once()

    device.client = disconnected_client
    await device.disconnect()
    disconnected_client.disconnect.assert_not_called()
