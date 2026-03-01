import struct
from unittest.mock import AsyncMock

import numpy as np
import pytest

import biocoin.techniques.base_technique as base_module
from biocoin.techniques import (
    ChronoAmperometry,
    CyclicVoltammetry,
    DifferentialPulseVoltammetry,
    Impedance,
    SquareWaveVoltammetry,
)


@pytest.mark.asyncio
async def test_ca_notification_handler_parses_chunked_float(technique_device) -> None:
    ca = ChronoAmperometry(technique_device)
    payload = struct.pack('<f', -12.5)

    await ca.notification_handler(0, payload[:2])
    assert ca.data_queue.empty()

    await ca.notification_handler(0, payload[2:])
    assert ca.data_queue.get_nowait() == pytest.approx(-12.5)


@pytest.mark.asyncio
async def test_cv_notification_handler_parses_multiple_floats(technique_device) -> None:
    cv = CyclicVoltammetry(technique_device)
    payload = struct.pack('<ff', 1.25, -2.5)

    await cv.notification_handler(0, payload)

    assert cv.data_queue.get_nowait() == pytest.approx(1.25)
    assert cv.data_queue.get_nowait() == pytest.approx(-2.5)


@pytest.mark.asyncio
async def test_impedance_notification_handler_parses_mag_phase_and_converts_to_degrees(technique_device) -> None:
    imp = Impedance(technique_device)
    payload = struct.pack('<ff', 123.0, np.pi / 2)

    await imp.notification_handler(0, payload)

    magnitude, phase_deg = imp.data_queue.get_nowait()
    assert magnitude == pytest.approx(123.0)
    assert phase_deg == pytest.approx(90.0)


@pytest.mark.asyncio
async def test_dpv_run_pairwise_difference_and_odd_sample_drop(
    monkeypatch: pytest.MonkeyPatch, technique_device
) -> None:
    async def no_sleep(_: float) -> None:
        return None

    monkeypatch.setattr(base_module.asyncio, 'sleep', no_sleep)

    dpv = DifferentialPulseVoltammetry(technique_device)
    dpv.start = AsyncMock()
    dpv.stop = AsyncMock()
    dpv.is_done = AsyncMock(return_value=True)
    dpv.duration = 0.0
    dpv.V = np.array([50.0])
    dpv.data_queue.put_nowait(10.0)
    dpv.data_queue.put_nowait(7.0)
    dpv.data_queue.put_nowait(99.0)  # odd value should be dropped

    result = await dpv.run()

    assert result.shape == (1, 2)
    assert result[0, 0] == pytest.approx(50.0)
    assert result[0, 1] == pytest.approx(3.0)


@pytest.mark.asyncio
async def test_swv_run_pairwise_difference_uses_second_minus_first(
    monkeypatch: pytest.MonkeyPatch, technique_device
) -> None:
    async def no_sleep(_: float) -> None:
        return None

    monkeypatch.setattr(base_module.asyncio, 'sleep', no_sleep)

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
