"""
Shared helpers for pulse-style voltammetry techniques (DPV/SWV).
"""

from collections.abc import Callable
from typing import cast

import numpy as np
from numpy.typing import NDArray

from utils.logging_util import get_logger

logger = get_logger(__name__)


def calc_staircase_voltage_vector(
    segment_builder: Callable[[float, float, float], np.ndarray],
    *,
    E_start: float,
    E_stop: float,
    E_step: float,
) -> np.ndarray:
    """
    Build a staircase voltage vector from ``E_start`` to ``E_stop`` using ``E_step``.

    Parameters:
        - segment_builder (Callable[[float, float, float], np.ndarray]): Segment generator callback.
        - E_start (float): Start potential in mV.
        - E_stop (float): Stop potential in mV.
        - E_step (float): Step size in mV.
    Returns:
        - np.ndarray: Staircase voltage vector.
    """
    if E_step == 0:
        raise ValueError('Estep quantizes to 0 mV; increase Estep.')
    return segment_builder(E_start, E_stop, E_step).astype(float, copy=False)


def pairwise_differential_current(samples: list[float], *, second_minus_first: bool, label: str) -> np.ndarray:
    """
    Compute pairwise current differences from interleaved half-cycle samples.

    Parameters:
        - samples (list[float]): Interleaved half-cycle current samples.
        - second_minus_first (bool): Select subtraction order for each pair.
        - label (str): Technique label for warning logs.
    Returns:
        - np.ndarray: Differential current values (one per sample pair).
    """
    num_pairs = len(samples) // 2
    if len(samples) % 2 != 0:
        logger.warning(f'Odd number of {label} samples received; dropping last sample.')

    packed: NDArray[np.float64] = np.asarray(samples[: 2 * num_pairs], dtype=np.float64)
    if second_minus_first:
        return cast(NDArray[np.float64], packed[1::2] - packed[0::2])
    return cast(NDArray[np.float64], packed[0::2] - packed[1::2])


