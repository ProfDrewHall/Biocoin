import numpy as np
import pytest

from biocoin.techniques import CyclicVoltammetry, DifferentialPulseVoltammetry, SquareWaveVoltammetry


def test_cv_voltage_vector_nominal_cycle(minimal_device) -> None:
    cv = CyclicVoltammetry(minimal_device)

    actual = cv.calc_voltage_vector(E_start=0.0, E_vertex1=100.0, E_vertex2=-100.0, E_step=50.0, cycles=1)
    expected = np.array([0.0, 50.0, 100.0, 50.0, 0.0, -50.0, -100.0, -50.0])

    assert np.allclose(actual, expected)


def test_cv_voltage_vector_validation_errors(minimal_device) -> None:
    cv = CyclicVoltammetry(minimal_device)

    with pytest.raises(ValueError, match=r'\|E_step\| must be >'):
        cv.calc_voltage_vector(E_start=0.0, E_vertex1=100.0, E_vertex2=-100.0, E_step=0.1, cycles=1)

    with pytest.raises(ValueError, match='cycles must be'):
        cv.calc_voltage_vector(E_start=0.0, E_vertex1=100.0, E_vertex2=-100.0, E_step=50.0, cycles=0)


@pytest.mark.parametrize(
    ('technique_cls', 'kwargs', 'expected'),
    [
        (
            DifferentialPulseVoltammetry,
            {'E_start': -100.0, 'E_stop': 100.0, 'E_step': 50.0},
            np.array([-100.0, -50.0, 0.0, 50.0, 100.0]),
        ),
        (
            DifferentialPulseVoltammetry,
            {'E_start': 100.0, 'E_stop': -100.0, 'E_step': 50.0},
            np.array([100.0, 50.0, 0.0, -50.0, -100.0]),
        ),
        (
            SquareWaveVoltammetry,
            {'E_start': -100.0, 'E_stop': 100.0, 'E_step': 50.0},
            np.array([-100.0, -50.0, 0.0, 50.0, 100.0]),
        ),
        (
            SquareWaveVoltammetry,
            {'E_start': 100.0, 'E_stop': -100.0, 'E_step': 50.0},
            np.array([100.0, 50.0, 0.0, -50.0, -100.0]),
        ),
    ],
)
def test_dpv_swv_voltage_vectors(technique_cls, kwargs: dict, expected: np.ndarray, minimal_device) -> None:
    technique = technique_cls(minimal_device)

    actual = technique.calc_voltage_vector(**kwargs)

    assert np.allclose(actual, expected)


@pytest.mark.parametrize('technique_cls', [DifferentialPulseVoltammetry, SquareWaveVoltammetry])
def test_dpv_swv_voltage_vector_rejects_zero_step(technique_cls, minimal_device) -> None:
    technique = technique_cls(minimal_device)

    with pytest.raises(ValueError, match='Estep quantizes to 0 mV'):
        technique.calc_voltage_vector(E_start=0.0, E_stop=100.0, E_step=0.0)
