import struct
from unittest.mock import AsyncMock

import pytest

from biocoin.techniques import (
    ChronoAmperometry,
    CVConfig,
    CyclicVoltammetry,
    DifferentialPulseVoltammetry,
    Impedance,
    Iontophoresis,
    OpenCircuitPotential,
    SquareWaveVoltammetry,
    Temperature,
)


@pytest.mark.asyncio
async def test_ca_config_payload_layout(technique_device) -> None:
    device = technique_device
    ca = ChronoAmperometry(device)
    ca.assert_config_ok = AsyncMock()

    await ca.configure(
        sampling_interval=0.2,
        processing_interval=0.5,
        max_current=100.0,
        pulse_potential=250.0,
        channel=2,
    )

    payload = device.write_technique_config.await_args.args[0]
    unpacked = struct.unpack('<BffffB', payload)
    assert unpacked[0] == int(ca.Technique.CA)
    assert unpacked[1:] == pytest.approx((0.2, 0.5, 100.0, 250.0, 2))


@pytest.mark.asyncio
async def test_cv_config_payload_layout_and_duration(technique_device) -> None:
    device = technique_device
    cv = CyclicVoltammetry(device)
    cv.assert_config_ok = AsyncMock()

    await cv.configure(
        processing_interval=0.2,
        max_current=200.0,
        E_start=0.0,
        E_vertex1=200.0,
        E_vertex2=-200.0,
        E_step=50.0,
        pulse_width=20.0,
        channel=1,
    )

    payload = device.write_technique_config.await_args.args[0]
    unpacked = struct.unpack('<BfffffffB', payload)
    assert unpacked[0] == int(cv.Technique.CV)
    assert unpacked[1:] == pytest.approx((0.2, 200.0, 0.0, 200.0, -200.0, 50.0, 20.0, 1))
    assert cv.duration >= 2.0
    assert cv.V is not None
    assert len(cv.V) > 0


def test_cv_config_dataclass_validate_and_payload() -> None:
    config = CVConfig(
        processing_interval=0.2,
        max_current=200.0,
        E_start=0.0,
        E_vertex1=200.0,
        E_vertex2=-200.0,
        E_step=50.0,
        pulse_width=20.0,
        channel=1,
    )
    config.validate()
    payload = config.to_payload(0x02)
    unpacked = struct.unpack('<BfffffffB', payload)
    assert unpacked[0] == 0x02
    assert unpacked[1:] == pytest.approx((0.2, 200.0, 0.0, 200.0, -200.0, 50.0, 20.0, 1))


@pytest.mark.asyncio
async def test_dpv_config_payload_layout_and_duration(technique_device) -> None:
    device = technique_device
    dpv = DifferentialPulseVoltammetry(device)
    dpv.assert_config_ok = AsyncMock()

    await dpv.configure(
        processing_interval=0.2,
        max_current=200.0,
        E_start=-200.0,
        E_stop=200.0,
        E_pulse=100.0,
        E_step=50.0,
        pulse_width=50.0,
        pulse_period=200.0,
        channel=3,
    )

    payload = device.write_technique_config.await_args.args[0]
    unpacked = struct.unpack('<BffffffffB', payload)
    assert unpacked[0] == int(dpv.Technique.DPV)
    assert unpacked[1:] == pytest.approx((0.2, 200.0, -200.0, 200.0, 100.0, 50.0, 50.0, 200.0, 3))
    assert dpv.duration >= 2.0
    assert dpv.V is not None


@pytest.mark.asyncio
async def test_swv_config_payload_layout_and_duration(technique_device) -> None:
    device = technique_device
    swv = SquareWaveVoltammetry(device)
    swv.assert_config_ok = AsyncMock()

    await swv.configure(
        processing_interval=0.2,
        max_current=200.0,
        E_start=-200.0,
        E_stop=200.0,
        E_amplitude=100.0,
        E_step=50.0,
        pulse_period=100.0,
        channel=0,
    )

    payload = device.write_technique_config.await_args.args[0]
    unpacked = struct.unpack('<BfffffffB', payload)
    assert unpacked[0] == int(swv.Technique.SWV)
    assert unpacked[1:] == pytest.approx((0.2, 200.0, -200.0, 200.0, 100.0, 50.0, 100.0, 0))
    assert swv.duration >= 2.0
    assert swv.V is not None


@pytest.mark.asyncio
async def test_ocp_channel_mapping_in_payload(technique_device) -> None:
    device = technique_device
    ocp = OpenCircuitPotential(device)
    ocp.assert_config_ok = AsyncMock()

    await ocp.configure(sampling_interval=0.25, processing_interval=0.5, channel=2)

    payload = device.write_technique_config.await_args.args[0]
    unpacked = struct.unpack('<BffB', payload)
    assert unpacked[0] == int(ocp.Technique.OCP)
    assert unpacked[1] == pytest.approx(0.25)
    assert unpacked[2] == pytest.approx(0.5)
    assert unpacked[3] == 0x10


@pytest.mark.asyncio
async def test_temp_channel_mapping_in_payload(technique_device) -> None:
    device = technique_device
    temp = Temperature(device)
    temp.assert_config_ok = AsyncMock()

    await temp.configure(sampling_interval=0.25, processing_interval=0.5, channel=1)

    payload = device.write_technique_config.await_args.args[0]
    unpacked = struct.unpack('<BffB', payload)
    assert unpacked[0] == int(temp.Technique.TEMP)
    assert unpacked[1] == pytest.approx(0.25)
    assert unpacked[2] == pytest.approx(0.5)
    assert unpacked[3] == 0x1D


@pytest.mark.asyncio
async def test_impedance_payload_layout_and_duration(technique_device) -> None:
    device = technique_device
    imp = Impedance(device)
    imp.assert_config_ok = AsyncMock()

    await imp.configure(
        sampling_interval=0.1,
        processing_interval=0.2,
        IMP_4wire=True,
        AC_coupled=False,
        max_current=300.0,
        E_ac=100.0,
        frequency=2.0,
    )

    payload = device.write_technique_config.await_args.args[0]
    unpacked = struct.unpack('<BffBBfff', payload)
    assert unpacked[0] == int(imp.Technique.IMP)
    assert unpacked[1:] == pytest.approx((0.1, 0.2, 1, 0, 300.0, 100.0, 2.0))
    assert imp.duration == pytest.approx(5.0)


@pytest.mark.asyncio
async def test_iontophoresis_payload_layout(technique_device) -> None:
    device = technique_device
    ion = Iontophoresis(device)
    ion.assert_config_ok = AsyncMock()

    await ion.configure(current_monitor_interval=0.5, stim_current=10.0, current_safety_threshold=20.0)

    payload = device.write_technique_config.await_args.args[0]
    unpacked = struct.unpack('<Bfff', payload)
    assert unpacked[0] == int(ion.Technique.IONTOPHORESIS)
    assert unpacked[1:] == pytest.approx((0.5, 10.0, 20.0))
