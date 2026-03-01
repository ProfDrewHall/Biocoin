import argparse
import asyncio
import logging

import numpy as np

from biocoin.device import BiocoinDevice
from biocoin.techniques import DifferentialPulseVoltammetry
from utils.logging_util import setup_logging

try:
    from tests.integration._validation_common import ValidationCase, assert_within_tolerance
except ModuleNotFoundError:
    try:
        from integration._validation_common import ValidationCase, assert_within_tolerance
    except ModuleNotFoundError:
        from _validation_common import ValidationCase, assert_within_tolerance


async def run_and_assert_dpv_accuracy(
    device: BiocoinDevice,
    base_config: dict,
    resistor_ohms: float,
    tolerance_fraction: float,
    label: str,
) -> None:
    dpv = DifferentialPulseVoltammetry(device)
    await dpv.configure(**base_config)
    data = await dpv.run()

    if dpv.V is None:
        raise AssertionError(f'[{label}] DPV voltage vector was not initialized')

    actual_points = len(data)
    expected_points = len(dpv.V)
    if actual_points != expected_points:
        raise AssertionError(f'[{label}] expected exactly {expected_points} points, got {actual_points}')
    if actual_points == 0:
        raise AssertionError(f'[{label}] DPV run returned no data points')

    voltage_mV = data[:, 0]
    current_uA = data[:, 1]
    expected_diff_uA = (base_config['E_pulse'] * 1000.0) / resistor_ohms
    mean_abs_current_uA = float(np.mean(np.abs(current_uA)))
    assert_within_tolerance(
        label=f'{label}/mean_abs_diff_current',
        actual=mean_abs_current_uA,
        expected=abs(expected_diff_uA),
        tolerance_fraction=tolerance_fraction,
        units=' uA',
    )

    expected_min_voltage = min(base_config['E_start'], base_config['E_stop'])
    expected_max_voltage = max(base_config['E_start'], base_config['E_stop'])
    voltage_margin = max(abs(base_config['E_step']), 2.0)
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
        '[PASS] %s: points=%d (expected %d), mean |diff current|=%.3f uA (expected %.3f uA)',
        label,
        actual_points,
        expected_points,
        mean_abs_current_uA,
        abs(expected_diff_uA),
    )


async def expect_validation_error(device: BiocoinDevice, base_config: dict, case: ValidationCase) -> None:
    config = dict(base_config)
    config.update(case.overrides)

    dpv = DifferentialPulseVoltammetry(device)
    try:
        await dpv.configure(**config)
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
    logging.info('Starting DPV 2-electrode resistor test + validation checks')

    device = BiocoinDevice()
    await device.connect(name=args.device_name)

    base_config = {
        'processing_interval': args.processing_interval,
        'max_current': args.max_current,
        'E_start': args.e_start,
        'E_stop': args.e_stop,
        'E_pulse': args.e_pulse,
        'E_step': args.e_step,
        'pulse_width': args.pulse_width,
        'pulse_period': args.pulse_period,
        'channel': args.channel,
    }

    try:
        # DPV runs in 2-electrode setup (fixed resistor between RE and WE):
        # 1) baseline validation run
        await run_and_assert_dpv_accuracy(
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
        await run_and_assert_dpv_accuracy(
            device=device,
            base_config=large_processing_config,
            resistor_ohms=args.resistor_ohms,
            tolerance_fraction=args.tolerance_fraction,
            label='large_processing_interval_ratio',
        )

        # 3) smaller voltage step than baseline
        small_step_config = dict(base_config)
        small_step_config['E_step'] = max(base_config['E_step'] * 0.5, 1.0)
        await run_and_assert_dpv_accuracy(
            device=device,
            base_config=small_step_config,
            resistor_ohms=args.resistor_ohms,
            tolerance_fraction=args.tolerance_fraction,
            label='small_step_interval',
        )

        validation_cases = [
            ValidationCase('processing_interval_zero', {'processing_interval': 0.0}, 'processing_interval must be > 0'),
            ValidationCase(
                'processing_interval_below_pulse_width',
                {'processing_interval': (base_config['pulse_width'] / 1000.0) * 0.5},
                'processing_interval must be',
            ),
            ValidationCase('max_current_out_of_range', {'max_current': 3001.0}, 'max_current must be > 0'),
            ValidationCase('e_start_out_of_bounds', {'E_start': 2201.0}, 'E_start must be between'),
            ValidationCase('e_stop_out_of_bounds', {'E_stop': -2201.0}, 'E_stop must be between'),
            ValidationCase('e_step_zero', {'E_step': 0.0}, 'E_step must be nonzero'),
            ValidationCase('e_pulse_nonpositive', {'E_pulse': 0.0}, 'E_pulse must be > 0'),
            ValidationCase('pulse_width_too_small', {'pulse_width': 2.0}, 'pulse_width must be between'),
            ValidationCase(
                'pulse_period_less_than_width',
                {'pulse_period': base_config['pulse_width'] * 0.5},
                'pulse_period must be',
            ),
            ValidationCase('channel_invalid', {'channel': 4}, 'channel must be an integer between 0 and 3'),
        ]

        for case in validation_cases:
            await expect_validation_error(device, base_config, case)

        logging.info('All DPV parameter validation checks passed.')
    finally:
        await device.disconnect()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Run DPV in 2-electrode resistor setup and verify parameter validation errors.'
    )
    parser.add_argument('--device-name', default='Biocoin')
    parser.add_argument('--channel', type=int, default=0, help='2-electrode channel index (default: 0)')
    parser.add_argument('--processing-interval', type=float, default=0.2, help='DPV processing interval in seconds')
    parser.add_argument('--max-current', type=float, default=200.0, help='DPV max current in uA')
    parser.add_argument('--e-start', type=float, default=-200.0, help='DPV start potential in mV')
    parser.add_argument('--e-stop', type=float, default=200.0, help='DPV stop potential in mV')
    parser.add_argument('--e-pulse', type=float, default=100.0, help='DPV pulse amplitude in mV')
    parser.add_argument('--e-step', type=float, default=50.0, help='DPV step size in mV')
    parser.add_argument('--pulse-width', type=float, default=50.0, help='DPV pulse width in ms')
    parser.add_argument('--pulse-period', type=float, default=200.0, help='DPV pulse period in ms')
    parser.add_argument('--resistor-ohms', type=float, default=10000.0, help='Fixed RE-WE resistor value in ohms')
    parser.add_argument(
        '--tolerance-fraction',
        type=float,
        default=0.20,
        help='Allowed relative error for DPV checks (default: 0.20)',
    )
    return parser


if __name__ == '__main__':
    parser = build_parser()
    asyncio.run(run_test(parser.parse_args()))


