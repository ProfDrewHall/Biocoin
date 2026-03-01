from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ValidationCase:
    name: str
    overrides: Mapping[str, Any]
    expected_error_substring: str


def assert_within_tolerance(
    label: str, actual: float, expected: float, tolerance_fraction: float, units: str
) -> None:
    if expected == 0:
        if actual != 0:
            raise AssertionError(f'[{label}] expected {expected}{units}, got {actual}{units}')
        return

    relative_error = abs(actual - expected) / abs(expected)
    if relative_error > tolerance_fraction:
        raise AssertionError(
            f'[{label}] expected {expected:.3f}{units}, got {actual:.3f}{units} '
            f'(error {relative_error * 100:.2f}% > {tolerance_fraction * 100:.1f}%)'
        )
