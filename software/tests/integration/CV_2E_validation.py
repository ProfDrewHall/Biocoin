import argparse
import asyncio
import logging

import numpy as np

from biocoin.device import BiocoinDevice
from biocoin.techniques import CyclicVoltammetry
from utils.logging_util import setup_logging

try:
    from tests.integration._validation_common import ValidationCase, assert_within_tolerance
except ModuleNotFoundError:
    try:
        from integration._validation_common import ValidationCase, assert_within_tolerance
    except ModuleNotFoundError:
        from _validation_common import ValidationCase, assert_within_tolerance


async def run_and_assert_cv_accuracy(
    device: BiocoinDevice,
    base_config: dict,
    resistor_ohms: float,
    tolerance_fraction: float,
    label: str,
) -> None:
    cv = CyclicVoltammetry(device)
    await cv.configure(**base_config)
    data = await cv.run()

    if cv.V is None:
        raise AssertionError(f'[{label}] CV voltage vector was not initialized')

    actual_points = len(data)
    expected_points = len(cv.V)
    if actual_points != expected_points:
        raise AssertionError(f'[{label}] expected exactly {expected_points} points, got {actual_points}')
    if actual_points == 0:
        raise AssertionError(f'[{label}] CV run returned no data points')

    voltage_mV = data[:, 0]
    current_uA = data[:, 1]

    expected_current_uA = voltage_mV * 1000.0 / resistor_ohms
    max_expected_uA = float(np.max(np.abs(expected_current_uA)))
    mean_abs_error_uA = float(np.mean(np.abs(current_uA - expected_current_uA)))
    allowed_mean_abs_error_uA = max(max_expected_uA * tolerance_fraction, 1.0)
    if mean_abs_error_uA > allowed_mean_abs_error_uA:
        raise AssertionError(
            f'[{label}] expected mean absolute current error <= {allowed_mean_abs_error_uA:.3f} uA, '
            f'got {mean_abs_error_uA:.3f} uA'
        )

    slope_uA_per_mV, intercept_uA = np.polyfit(voltage_mV, current_uA, 1)
    expected_slope_uA_per_mV = 1000.0 / resistor_ohms
    assert_within_tolerance(
        label=f'{label}/iv_slope',
        actual=float(slope_uA_per_mV),
        expected=expected_slope_uA_per_mV,
        tolerance_fraction=tolerance_fraction,
        units=' uA/mV',
    )
    if abs(float(intercept_uA)) > allowed_mean_abs_error_uA:
        raise AssertionError(
            f'[{label}] expected |I-V intercept| <= {allowed_mean_abs_error_uA:.3f} uA, '
            f'got {float(intercept_uA):.3f} uA'
        )

    expected_min_voltage = min(base_config['E_start'], base_config['E_vertex1'], base_config['E_vertex2'])
    expected_max_voltage = max(base_config['E_start'], base_config['E_vertex1'], base_config['E_vertex2'])
    voltage_margin = max(base_config['E_step'], 2.0)
    observed_min_voltage = float(np.min(voltage_mV))
    observed_max_voltage = float(np.max(voltage_mV))
    if observed_min_voltage > expected_min_voltage + voltage_margin:
        raise AssertionError(
            f'[{label}] voltage sweep min too high: expected <= {expected_min_voltage + voltage_margin:.3f} mV, '
            f'got {observed_min_voltage:.3f} mV'
        )
    if observed_max_voltage < expected_max_voltage - voltage_margin:
        raise AssertionError(
            f'[{label}] voltage sweep max too low: expected >= {expected_max_voltage - voltage_margin:.3f} mV, '
            f'got {observed_max_voltage:.3f} mV'
        )

    logging.info(
        '[PASS] %s: points=%d (expected %d), slope=%.6f uA/mV (expected %.6f), MAE=%.3f uA',
        label,
        actual_points,
        expected_points,
        slope_uA_per_mV,
        expected_slope_uA_per_mV,
        mean_abs_error_uA,
    )


async def expect_validation_error(device: BiocoinDevice, base_config: dict, case: ValidationCase) -> None:
    config = dict(base_config)
    config.update(case.overrides)

    cv = CyclicVoltammetry(device)
    try:
        await cv.configure(**config)
    except ValueError as exc:
        message = str(exc)
        if case.expected_error_substring not in message:
            raise AssertionError(
                f'[{case.name}] expected error containing "{case.expected_error_substring}", got "{message}"'
            ) from exc
        logging.info(f'[PASS] {case.name} -> {message}')
        return

    raise AssertionError(f'[{case.name}] expected ValueError but configure() succeeded')


async def run_test(args: argparse.Namespace) -> None:
    setup_logging()
    logging.info('Starting CV 2-electrode resistor test + validation checks')

    device = BiocoinDevice()
    await device.connect(name=args.device_name)

    base_config = {
        'processing_interval': args.processing_interval,
        'max_current': args.max_current,
        'E_start': args.e_start,
        'E_vertex1': args.e_vertex1,
        'E_vertex2': args.e_vertex2,
        'E_step': args.e_step,
        'pulse_width': args.pulse_width,
        'channel': args.channel,
    }

    try:
        # CV runs in 2-electrode setup (fixed resistor between RE and WE):
        # 1) baseline validation run
        await run_and_assert_cv_accuracy(
            device=device,
            base_config=base_config,
            resistor_ohms=args.resistor_ohms,
            tolerance_fraction=args.tolerance_fraction,
            label='baseline_validation',
        )

        # 2) larger processing interval relative to pulse width
        large_processing_config = dict(base_config)
        large_processing_config['processing_interval'] = max(
            base_config['processing_interval'],
            (base_config['pulse_width'] / 1000.0) * 5.0,
        )
        await run_and_assert_cv_accuracy(
            device=device,
            base_config=large_processing_config,
            resistor_ohms=args.resistor_ohms,
            tolerance_fraction=args.tolerance_fraction,
            label='large_processing_interval_ratio',
        )

        # 3) smaller potential step than baseline
        small_step_config = dict(base_config)
        small_step_config['E_step'] = max(base_config['E_step'] * 0.5, 1.0)
        await run_and_assert_cv_accuracy(
            device=device,
            base_config=small_step_config,
            resistor_ohms=args.resistor_ohms,
            tolerance_fraction=args.tolerance_fraction,
            label='small_step_interval',
        )

        validation_cases = [
            ValidationCase(
                'processing_interval_below_pulse_width',
                {'processing_interval': (base_config['pulse_width'] / 1000.0) * 0.5},
                'processing_interval must be >= pulse_width in seconds',
            ),
            ValidationCase('max_current_out_of_range', {'max_current': 3001.0}, 'max_current must be > 0'),
            ValidationCase('E_step_too_small', {'E_step': 0.5}, 'E_step must be between'),
            ValidationCase('pulse_width_too_small', {'pulse_width': 2.0}, 'pulse_width must be between'),
            ValidationCase('channel_invalid', {'channel': 4}, 'channel must be an integer between 0 and 3'),
            ValidationCase(
                'E_start_not_between_vertices',
                {'E_start': 400.0, 'E_vertex1': -200.0, 'E_vertex2': 200.0},
                'E_start must be bounded between E_vertex1 and E_vertex2',
            ),
            ValidationCase(
                'vertex_span_too_large',
                {'E_vertex1': -1200.0, 'E_vertex2': 1201.0},
                'must be within 2200 mV',
            ),
            ValidationCase('vertex_out_of_bounds', {'E_vertex1': 2201.0}, 'E_vertex1 must be between'),
        ]

        for case in validation_cases:
            await expect_validation_error(device, base_config, case)

        logging.info('All CV parameter validation checks passed.')
    finally:
        await device.disconnect()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Run CV in 2-electrode resistor setup and verify parameter validation errors.'
    )
    parser.add_argument('--device-name', default='Biocoin')
    parser.add_argument('--channel', type=int, default=0, help='2-electrode channel index (default: 0)')
    parser.add_argument('--processing-interval', type=float, default=0.2, help='CV processing interval in seconds')
    parser.add_argument('--max-current', type=float, default=200.0, help='CV max current in uA')
    parser.add_argument('--e-start', type=float, default=0.0, help='CV start potential in mV')
    parser.add_argument('--e-vertex1', type=float, default=200.0, help='CV vertex 1 potential in mV')
    parser.add_argument('--e-vertex2', type=float, default=-200.0, help='CV vertex 2 potential in mV')
    parser.add_argument('--e-step', type=float, default=10.0, help='CV potential step in mV')
    parser.add_argument('--pulse-width', type=float, default=50.0, help='CV pulse width in ms')
    parser.add_argument('--resistor-ohms', type=float, default=10000.0, help='Fixed RE-WE resistor value in ohms')
    parser.add_argument(
        '--tolerance-fraction',
        type=float,
        default=0.20,
        help='Allowed relative error for CV checks (default: 0.20)',
    )
    return parser


if __name__ == '__main__':
    parser = build_parser()
    asyncio.run(run_test(parser.parse_args()))


