"""
Standalone IMP example CLI for running impedance spectroscopy with explicit parameters.
"""

import argparse
import asyncio
from pathlib import Path

import numpy as np

from biocoin.device import BiocoinDevice
from biocoin.errors import BiocoinError, ConfigError, DeviceStatusError, TransportError
from biocoin.techniques import Impedance
from utils.logging_util import get_logger, setup_logging

logger = get_logger(__name__)
DEFAULT_DURATION = 15
DEFAULT_SAMPLING_INTERVAL = 0.1
DEFAULT_PROCESSING_INTERVAL = 0.2
DEFAULT_IMP_4WIRE = True
DEFAULT_AC_COUPLED = False
DEFAULT_MAX_CURRENT = 300.0
DEFAULT_E_AC = 100.0
DEFAULT_FREQUENCY = 2.0
EXPECTED_RUNTIME_ERROR_PREFIXES: tuple[str, ...] = (
    'BLE client ',
    'Biocoin device named',
    'Failed to connect to the Biocoin device',
    'Connected returned but client.is_connected is False',
    'Device reported ',
    'Expected ',
)


def is_expected_runtime_error(exc: RuntimeError) -> bool:
    """
    Determine whether a RuntimeError represents a known device/technique failure.

    Parameters:
        - exc (RuntimeError): RuntimeError to classify.
    Returns:
        - bool: True when the error should map to DeviceStatusError.
    """
    return str(exc).startswith(EXPECTED_RUNTIME_ERROR_PREFIXES)


def write_csv(data: np.ndarray, output_path: Path) -> None:
    """
    Write IMP output to CSV.

    Parameters:
        - data (np.ndarray): 2D array with [magnitude (Ohm), phase (deg)] columns.
        - output_path (Path): Destination CSV file.
    Returns:
        - None
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(
        output_path,
        np.asarray(data, dtype=float),
        delimiter=',',
        header='Magnitude (Ohm), Phase (deg)',
        comments='',
    )


def build_parser() -> argparse.ArgumentParser:
    """
    Build parser for standalone IMP example runner.

    Parameters:
        - None
    Returns:
        - argparse.ArgumentParser: Configured parser.
    """
    parser = argparse.ArgumentParser(description='Run IMP with explicit per-technique parameters.')
    parser.add_argument('--device-name', default='Biocoin', help='BLE device name to connect to.')
    parser.add_argument('--output', type=Path, default=Path('./results/IMP_output.csv'), help='Output CSV path.')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose console logging.')
    parser.add_argument('--duration', type=int, default=DEFAULT_DURATION, help='Run duration (s).')
    parser.add_argument(
        '--sampling-interval',
        type=float,
        default=DEFAULT_SAMPLING_INTERVAL,
        help='Time between samples (s).',
    )
    parser.add_argument(
        '--processing-interval',
        type=float,
        default=DEFAULT_PROCESSING_INTERVAL,
        help='Time between processing interrupts (s).',
    )
    parser.add_argument(
        '--imp-4wire',
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_IMP_4WIRE,
        help='Enable or disable 4-wire impedance mode.',
    )
    parser.add_argument(
        '--ac-coupled',
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_AC_COUPLED,
        help='Enable or disable AC coupling.',
    )
    parser.add_argument(
        '--max-current',
        type=float,
        default=DEFAULT_MAX_CURRENT,
        help='Maximum current (uA).',
    )
    parser.add_argument('--e-ac', type=float, default=DEFAULT_E_AC, help='AC excitation amplitude (mV).')
    parser.add_argument('--frequency', type=float, default=DEFAULT_FREQUENCY, help='Excitation frequency (Hz).')
    return parser


async def run(args: argparse.Namespace) -> None:
    """
    Execute IMP technique with provided CLI arguments.

    Parameters:
        - args (argparse.Namespace): Parsed CLI arguments.
    Returns:
        - None
    """
    setup_logging(verbose=args.verbose)
    logger.info('Starting standalone IMP run.')

    device = BiocoinDevice()
    try:
        await device.connect(name=args.device_name)
        battery_level = await device.get_battery_level()
        logger.info(f'Battery level: {battery_level}%')

        imp = Impedance(device)
        await imp.configure(
            sampling_interval=args.sampling_interval,
            processing_interval=args.processing_interval,
            IMP_4wire=args.imp_4wire,
            AC_coupled=args.ac_coupled,
            max_current=args.max_current,
            E_ac=args.e_ac,
            frequency=args.frequency,
        )
        data = await imp.run(duration=args.duration)
        write_csv(data, args.output)
        logger.info(f'Saved {len(data)} IMP points to {args.output}')
    except BiocoinError:
        raise
    except TimeoutError as exc:
        raise TransportError('Connection timed out. Ensure the device is powered on and in range.') from exc
    except LookupError as exc:
        raise TransportError(f'Failed to access required BLE characteristic: {exc}') from exc
    except ValueError as exc:
        raise ConfigError(str(exc)) from exc
    except RuntimeError as exc:
        if is_expected_runtime_error(exc):
            raise DeviceStatusError(str(exc)) from exc
        raise
    finally:
        try:
            await device.disconnect()
        except Exception as exc:
            logger.warning(f'Failed to disconnect cleanly: {exc}')


def main() -> None:
    """
    Parse CLI arguments and execute the async IMP run.

    Parameters:
        - None
    Returns:
        - None
    """
    parser = build_parser()
    args = parser.parse_args()
    try:
        asyncio.run(run(args))
    except BiocoinError as exc:
        logger.exception('IMP run failed.')
        raise SystemExit(1) from exc


if __name__ == '__main__':
    main()
