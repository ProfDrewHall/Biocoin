import argparse
import asyncio
import logging

import numpy as np

from biocoin.device import BiocoinDevice
from biocoin.techniques import ChronoAmperometry
from utils.logging_util import setup_logging

try:
    from tests.integration._validation_common import ValidationCase, assert_within_tolerance
except ModuleNotFoundError:
    try:
        from integration._validation_common import ValidationCase, assert_within_tolerance
    except ModuleNotFoundError:
        from _validation_common import ValidationCase, assert_within_tolerance


async def run_and_assert_ca_accuracy(
    device: BiocoinDevice,
    base_config: dict,
    duration: float,
    resistor_ohms: float,
    tolerance_fraction: float,
    label: str,
) -> None:
    ca = ChronoAmperometry(device)
    await ca.configure(**base_config)
    data = await ca.run(duration=int(duration))

    actual_points = len(data)
    expected_points = duration / base_config['sampling_interval']
    if expected_points <= 0:
        raise AssertionError(f'[{label}] invalid expected point count: {expected_points}')

    if actual_points == 0:
        raise AssertionError(f'[{label}] CA run returned no data points')

    assert_within_tolerance(
        label=f'{label}/point_count',
        actual=actual_points,
        expected=expected_points,
        tolerance_fraction=tolerance_fraction,
        units=' pts',
    )

    current_uA = data[:, 1]
    mean_current_uA = float(np.mean(current_uA))
    expected_uA = (base_config['pulse_potential'] * 1000.0) / resistor_ohms
    assert_within_tolerance(
        label=f'{label}/mean_current',
        actual=mean_current_uA,
        expected=expected_uA,
        tolerance_fraction=tolerance_fraction,
        units=' uA',
    )

    logging.info(
        '[PASS] %s: points=%d (expected %.1f), mean current=%.3f uA (expected %.3f uA)',
        label,
        actual_points,
        expected_points,
        mean_current_uA,
        expected_uA,
    )


async def expect_validation_error(device: BiocoinDevice, base_config: dict, case: ValidationCase) -> None:
    config = dict(base_config)
    config.update(case.overrides)

    ca = ChronoAmperometry(device)
    try:
        await ca.configure(**config)
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
    logging.info('Starting CA 2-electrode resistor test + validation checks')

    device = BiocoinDevice()
    await device.connect(name=args.device_name)

    base_config = {
        'sampling_interval': args.sampling_interval,
        'processing_interval': args.processing_interval,
        'max_current': args.max_current,
        'pulse_potential': args.pulse_potential,
        'channel': args.channel,
    }

    try:
        # CA runs in 2-electrode setup (fixed resistor between RE and WE):
        # 1) quick validation run
        # 2) stress run (~100s by default)
        await run_and_assert_ca_accuracy(
            device=device,
            base_config=base_config,
            duration=args.quick_duration,
            resistor_ohms=args.resistor_ohms,
            tolerance_fraction=args.tolerance_fraction,
            label='quick_validation',
        )
        await run_and_assert_ca_accuracy(
            device=device,
            base_config=base_config,
            duration=args.stress_duration,
            resistor_ohms=args.resistor_ohms,
            tolerance_fraction=args.tolerance_fraction,
            label='stress_validation',
        )

        # Additional coverage:
        # 3) larger processing interval relative to sampling interval
        large_processing_config = dict(base_config)
        large_processing_config['processing_interval'] = max(
            base_config['processing_interval'],
            base_config['sampling_interval'] * 5.0,
        )
        await run_and_assert_ca_accuracy(
            device=device,
            base_config=large_processing_config,
            duration=args.quick_duration,
            resistor_ohms=args.resistor_ohms,
            tolerance_fraction=args.tolerance_fraction,
            label='large_processing_interval_ratio',
        )

        # 4) smaller sampling interval than baseline
        small_sampling_config = dict(base_config)
        small_sampling_config['sampling_interval'] = max(base_config['sampling_interval'] * 0.5, 0.05)
        await run_and_assert_ca_accuracy(
            device=device,
            base_config=small_sampling_config,
            duration=args.quick_duration,
            resistor_ohms=args.resistor_ohms,
            tolerance_fraction=args.tolerance_fraction,
            label='small_sampling_interval',
        )

        validation_cases = [
            ValidationCase('sampling_interval_zero', {'sampling_interval': 0.0}, 'sampling_interval must be > 0'),
            ValidationCase(
                'processing_interval_negative',
                {'processing_interval': -0.1},
                'processing_interval must be > 0',
            ),
            ValidationCase(
                'max_current_out_of_range',
                {'max_current': 10_001.0},
                'max_current must be > 0 and <= 10,000 uA',
            ),
            ValidationCase(
                'pulse_potential_out_of_range',
                {'pulse_potential': 1001.0},
                'pulse_potential must be between -1000 and +1000 mV',
            ),
            ValidationCase('channel_invalid', {'channel': 4}, 'channel must be an integer between 0 and 3'),
        ]

        for case in validation_cases:
            await expect_validation_error(device, base_config, case)

        logging.info('All CA parameter validation checks passed.')

    finally:
        await device.disconnect()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Run CA in 2-electrode resistor setup and verify parameter validation errors.'
    )
    parser.add_argument('--device-name', default='Biocoin')
    parser.add_argument('--channel', type=int, default=0, help='2-electrode channel index (default: 0)')
    parser.add_argument('--sampling-interval', type=float, default=1, help='CA sampling interval in seconds')
    parser.add_argument('--processing-interval', type=float, default=5, help='CA processing interval in seconds')
    parser.add_argument('--max-current', type=float, default=100.0, help='CA max current in uA')
    parser.add_argument('--pulse-potential', type=float, default=200.0, help='CA pulse potential in mV')
    parser.add_argument('--quick-duration', type=float, default=30.0, help='Quick CA run duration in seconds')
    parser.add_argument('--stress-duration', type=float, default=120.0, help='Stress-test CA run duration in seconds')
    parser.add_argument('--resistor-ohms', type=float, default=10000.0, help='Fixed RE-WE resistor value in ohms')
    parser.add_argument(
        '--tolerance-fraction',
        type=float,
        default=0.20,
        help='Allowed relative error for point count and mean current checks (default: 0.20)',
    )
    return parser


if __name__ == '__main__':
    parser = build_parser()
    asyncio.run(run_test(parser.parse_args()))
