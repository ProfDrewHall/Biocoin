import asyncio
from unittest.mock import AsyncMock

import numpy as np
import pytest

from biocoin.techniques.base_technique import BaseTechnique


class DummyTechnique(BaseTechnique):
    async def configure(self, **kwargs) -> None:  # pragma: no cover - trivial override for abstract base class
        return None

    async def run(self, **kwargs) -> np.ndarray:  # pragma: no cover - trivial override for abstract base class
        return np.empty((0, 2), dtype=float)

    async def notification_handler(self, sender: int, data: bytes) -> None:  # pragma: no cover - not used
        return None


@pytest.mark.asyncio
async def test_start_clears_queue_and_registers_notify_handler(technique_device) -> None:
    technique = DummyTechnique(technique_device)
    technique.data_queue.put_nowait(123.0)

    await technique.start()

    assert technique.data_queue.empty()
    technique_device.start_notify.assert_awaited_once()
    char_uuid, handler = technique_device.start_notify.await_args.args
    assert char_uuid == technique.char_uuid
    assert callable(handler)


@pytest.mark.asyncio
async def test_start_propagates_notification_registration_exception(technique_device) -> None:
    technique_device.start_notify.side_effect = RuntimeError('notify failed')
    technique = DummyTechnique(technique_device)

    with pytest.raises(RuntimeError, match='notify failed'):
        await technique.start()


@pytest.mark.asyncio
async def test_stop_forwards_to_device(technique_device) -> None:
    technique = DummyTechnique(technique_device)

    await technique.stop()

    technique_device.stop_notify.assert_awaited_once_with(technique.char_uuid)


def test_segment_forward_reverse_equal_and_zero_step(technique_device) -> None:
    technique = DummyTechnique(technique_device)

    assert np.allclose(technique._segment(0.0, 1.0, 0.5), np.array([0.0, 0.5, 1.0]))
    assert np.allclose(technique._segment(1.0, 0.0, 0.5), np.array([1.0, 0.5, 0.0]))
    assert np.allclose(technique._segment(2.0, 2.0, 0.5), np.array([2.0]))

    with pytest.raises(ValueError, match='step must be nonzero'):
        technique._segment(0.0, 1.0, 0.0)


@pytest.mark.asyncio
async def test_get_status_returns_enum(technique_device) -> None:
    technique_device.client = AsyncMock()
    technique_device.client.is_connected = True
    technique_device.client.read_gatt_char = AsyncMock(return_value=bytes([BaseTechnique.Status.RUNNING]))
    technique = DummyTechnique(technique_device)

    status = await technique.get_status()

    assert status == BaseTechnique.Status.RUNNING


@pytest.mark.asyncio
async def test_get_status_raises_for_disconnected_client(technique_device) -> None:
    technique_device.client = AsyncMock()
    technique_device.client.is_connected = False
    technique = DummyTechnique(technique_device)

    with pytest.raises(RuntimeError, match='BLE client not connected'):
        await technique.get_status()


@pytest.mark.asyncio
async def test_get_status_raises_for_empty_response(technique_device) -> None:
    technique_device.client = AsyncMock()
    technique_device.client.is_connected = True
    technique_device.client.read_gatt_char = AsyncMock(return_value=b'')
    technique = DummyTechnique(technique_device)

    with pytest.raises(ValueError, match='Empty status response'):
        await technique.get_status()


@pytest.mark.asyncio
async def test_get_status_raises_for_unknown_value(technique_device) -> None:
    technique_device.client = AsyncMock()
    technique_device.client.is_connected = True
    technique_device.client.read_gatt_char = AsyncMock(return_value=b'\xAA')
    technique = DummyTechnique(technique_device)

    with pytest.raises(ValueError, match='Unknown status'):
        await technique.get_status()


def test_is_fault_status(technique_device) -> None:
    technique = DummyTechnique(technique_device)

    assert technique.is_fault_status(BaseTechnique.Status.INVALID_PARAMETERS)
    assert technique.is_fault_status(BaseTechnique.Status.ERROR)
    assert technique.is_fault_status(BaseTechnique.Status.CURRENT_LIMIT_EXCEEDED)
    assert not technique.is_fault_status(BaseTechnique.Status.RUNNING)
    assert not technique.is_fault_status(BaseTechnique.Status.NOT_RUNNING)


@pytest.mark.asyncio
async def test_assert_config_ok_raises_for_invalid_parameters(
    monkeypatch: pytest.MonkeyPatch, technique_device
) -> None:
    technique = DummyTechnique(technique_device)

    async def no_sleep(_: float) -> None:
        return None

    monkeypatch.setattr(asyncio, 'sleep', no_sleep)
    technique.get_status = AsyncMock(return_value=BaseTechnique.Status.INVALID_PARAMETERS)

    with pytest.raises(ValueError, match='Invalid parameters'):
        await technique.assert_config_ok()


@pytest.mark.asyncio
async def test_assert_config_ok_raises_for_error(monkeypatch: pytest.MonkeyPatch, technique_device) -> None:
    technique = DummyTechnique(technique_device)

    async def no_sleep(_: float) -> None:
        return None

    monkeypatch.setattr(asyncio, 'sleep', no_sleep)
    technique.get_status = AsyncMock(return_value=BaseTechnique.Status.ERROR)

    with pytest.raises(RuntimeError, match='Device reported ERROR'):
        await technique.assert_config_ok()


@pytest.mark.asyncio
async def test_assert_config_ok_raises_for_current_limit(monkeypatch: pytest.MonkeyPatch, technique_device) -> None:
    technique = DummyTechnique(technique_device)

    async def no_sleep(_: float) -> None:
        return None

    monkeypatch.setattr(asyncio, 'sleep', no_sleep)
    technique.get_status = AsyncMock(return_value=BaseTechnique.Status.CURRENT_LIMIT_EXCEEDED)

    with pytest.raises(RuntimeError, match='CURRENT_LIMIT_EXCEEDED'):
        await technique.assert_config_ok()
