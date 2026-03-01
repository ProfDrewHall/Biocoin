"""
Standalone CA example CLI for running chronoamperometry with explicit parameters.
"""

import argparse
import asyncio
from pathlib import Path

import numpy as np

from biocoin.device import BiocoinDevice
from biocoin.errors import BiocoinError, ConfigError, DeviceStatusError, TransportError
from biocoin.techniques import ChronoAmperometry
from utils.logging_util import get_logger, setup_logging

logger = get_logger(__name__)
DEFAULT_DURATION = 15
DEFAULT_SAMPLING_INTERVAL = 0.2
DEFAULT_PROCESSING_INTERVAL = 0.5
DEFAULT_MAX_CURRENT = 100.0
DEFAULT_PULSE_POTENTIAL = 250.0
DEFAULT_CHANNEL = 0
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
    Write CA output to CSV.

    Parameters:
        - data (np.ndarray): 2D array with [time (s), current (uA)] columns.
        - output_path (Path): Destination CSV file.
    Returns:
        - None
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(output_path, np.asarray(data, dtype=float), delimiter=',', header='Time (s), Current (uA)', comments='')


def build_parser() -> argparse.ArgumentParser:
    """
    Build parser for standalone CA example runner.

    Parameters:
        - None
    Returns:
        - argparse.ArgumentParser: Configured parser.
    """
    parser = argparse.ArgumentParser(description='Run CA with explicit per-technique parameters.')
    parser.add_argument('--device-name', default='Biocoin', help='BLE device name to connect to.')
    parser.add_argument('--output', type=Path, default=Path('./results/CA_output.csv'), help='Output CSV path.')
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
        '--max-current',
        type=float,
        default=DEFAULT_MAX_CURRENT,
        help='Maximum current (uA).',
    )
    parser.add_argument(
        '--pulse-potential',
        type=float,
        default=DEFAULT_PULSE_POTENTIAL,
        help='Pulse potential (mV).',
    )
    parser.add_argument(
        '--channel',
        type=int,
        default=DEFAULT_CHANNEL,
        help='Electrode channel index (0-3).',
    )
    return parser


async def run(args: argparse.Namespace) -> None:
    """
    Execute CA technique with provided CLI arguments.

    Parameters:
        - args (argparse.Namespace): Parsed CLI arguments.
    Returns:
        - None
    """
    setup_logging(verbose=args.verbose)
    logger.info('Starting standalone CA run.')

    device = BiocoinDevice()
    try:
        await device.connect(name=args.device_name)
        battery_level = await device.get_battery_level()
        logger.info(f'Battery level: {battery_level}%')

        ca = ChronoAmperometry(device)
        await ca.configure(
            sampling_interval=args.sampling_interval,
            processing_interval=args.processing_interval,
            max_current=args.max_current,
            pulse_potential=args.pulse_potential,
            channel=args.channel,
        )
        data = await ca.run(duration=args.duration)
        write_csv(data, args.output)
        logger.info(f'Saved {len(data)} CA points to {args.output}')
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
    Parse CLI arguments and execute the async CA run.

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
        logger.exception('CA run failed.')
        raise SystemExit(1) from exc


if __name__ == '__main__':
    main()
