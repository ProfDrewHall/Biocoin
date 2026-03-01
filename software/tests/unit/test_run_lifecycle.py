from unittest.mock import AsyncMock, call

import numpy as np
import pytest

import biocoin.techniques.base_technique as base_module
from biocoin.techniques import CyclicVoltammetry, DifferentialPulseVoltammetry, SquareWaveVoltammetry


async def _no_sleep(_: float) -> None:
    return None


@pytest.mark.asyncio
async def test_cv_run_success_calls_start_and_stop(monkeypatch: pytest.MonkeyPatch, technique_device) -> None:
    monkeypatch.setattr(base_module.asyncio, 'sleep', _no_sleep)
    cv = CyclicVoltammetry(technique_device)
    cv.start = AsyncMock()
    cv.stop = AsyncMock()
    cv.is_done = AsyncMock(return_value=True)
    cv.duration = 0.0
    cv.V = np.array([0.0, 50.0])
    cv.data_queue.put_nowait(1.0)
    cv.data_queue.put_nowait(2.0)

    result = await cv.run()

    assert result.shape == (2, 2)
    technique_device.write_ctrl_command.assert_awaited_once_with(cv.Command.START)
    cv.stop.assert_awaited_once()


@pytest.mark.asyncio
async def test_cv_run_timeout_sends_stop_and_raises(monkeypatch: pytest.MonkeyPatch, technique_device) -> None:
    monkeypatch.setattr(base_module.asyncio, 'sleep', _no_sleep)
    cv = CyclicVoltammetry(technique_device)
    cv.start = AsyncMock()
    cv.stop = AsyncMock()
    cv.is_done = AsyncMock(return_value=False)
    cv.duration = 0.0
    cv.V = np.array([0.0])

    with pytest.raises(TimeoutError, match='Device did not finish'):
        await cv.run()

    assert technique_device.write_ctrl_command.await_args_list == [call(cv.Command.START), call(cv.Command.STOP)]
    cv.stop.assert_awaited_once()


@pytest.mark.asyncio
async def test_dpv_run_success_calls_start_and_stop(monkeypatch: pytest.MonkeyPatch, technique_device) -> None:
    monkeypatch.setattr(base_module.asyncio, 'sleep', _no_sleep)
    dpv = DifferentialPulseVoltammetry(technique_device)
    dpv.start = AsyncMock()
    dpv.stop = AsyncMock()
    dpv.is_done = AsyncMock(return_value=True)
    dpv.duration = 0.0
    dpv.V = np.array([10.0, 20.0])
    dpv.data_queue.put_nowait(5.0)
    dpv.data_queue.put_nowait(2.0)
    dpv.data_queue.put_nowait(9.0)
    dpv.data_queue.put_nowait(4.0)

    result = await dpv.run()

    assert result.shape == (2, 2)
    assert result[:, 1] == pytest.approx(np.array([3.0, 5.0]))
    technique_device.write_ctrl_command.assert_awaited_once_with(dpv.Command.START)
    dpv.stop.assert_awaited_once()


@pytest.mark.asyncio
async def test_dpv_run_timeout_sends_stop_and_raises(monkeypatch: pytest.MonkeyPatch, technique_device) -> None:
    monkeypatch.setattr(base_module.asyncio, 'sleep', _no_sleep)
    dpv = DifferentialPulseVoltammetry(technique_device)
    dpv.start = AsyncMock()
    dpv.stop = AsyncMock()
    dpv.is_done = AsyncMock(return_value=False)
    dpv.duration = 0.0
    dpv.V = np.array([0.0])

    with pytest.raises(TimeoutError, match='Device did not finish'):
        await dpv.run()

    assert technique_device.write_ctrl_command.await_args_list == [call(dpv.Command.START), call(dpv.Command.STOP)]
    dpv.stop.assert_awaited_once()


@pytest.mark.asyncio
async def test_swv_run_success_calls_start_and_stop(monkeypatch: pytest.MonkeyPatch, technique_device) -> None:
    monkeypatch.setattr(base_module.asyncio, 'sleep', _no_sleep)
    swv = SquareWaveVoltammetry(technique_device)
    swv.start = AsyncMock()
    swv.stop = AsyncMock()
    swv.is_done = AsyncMock(return_value=True)
    swv.duration = 0.0
    swv.V = np.array([100.0, 150.0])
    swv.data_queue.put_nowait(-9.15014)
    swv.data_queue.put_nowait(9.19441)
    swv.data_queue.put_nowait(-4.54272)
    swv.data_queue.put_nowait(13.78821)

    result = await swv.run()

    assert result.shape == (2, 2)
    assert result[:, 1] == pytest.approx(np.array([18.34455, 18.33093]), rel=1e-6, abs=1e-6)
    technique_device.write_ctrl_command.assert_awaited_once_with(swv.Command.START)
    swv.stop.assert_awaited_once()


@pytest.mark.asyncio
async def test_swv_run_timeout_sends_stop_and_raises(monkeypatch: pytest.MonkeyPatch, technique_device) -> None:
    monkeypatch.setattr(base_module.asyncio, 'sleep', _no_sleep)
    swv = SquareWaveVoltammetry(technique_device)
    swv.start = AsyncMock()
    swv.stop = AsyncMock()
    swv.is_done = AsyncMock(return_value=False)
    swv.duration = 0.0
    swv.V = np.array([0.0])

    with pytest.raises(TimeoutError, match='Device did not finish'):
        await swv.run()

    assert technique_device.write_ctrl_command.await_args_list == [call(swv.Command.START), call(swv.Command.STOP)]
    swv.stop.assert_awaited_once()
