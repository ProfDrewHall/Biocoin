"""
Standalone CV example CLI for running cyclic voltammetry with explicit parameters.
"""

import argparse
import asyncio
from pathlib import Path

import numpy as np

from biocoin.device import BiocoinDevice
from biocoin.errors import BiocoinError, ConfigError, DeviceStatusError, TransportError
from biocoin.techniques import CyclicVoltammetry
from utils.logging_util import get_logger, setup_logging

logger = get_logger(__name__)
DEFAULT_PROCESSING_INTERVAL = 0.2
DEFAULT_MAX_CURRENT = 200.0
DEFAULT_E_START = 0.0
DEFAULT_E_VERTEX1 = 200.0
DEFAULT_E_VERTEX2 = -200.0
DEFAULT_E_STEP = 50.0
DEFAULT_PULSE_WIDTH = 20.0
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
    Write CV output to CSV.

    Parameters:
        - data (np.ndarray): 2D array with [voltage (mV), current (uA)] columns.
        - output_path (Path): Destination CSV file.
    Returns:
        - None
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(
        output_path,
        np.asarray(data, dtype=float),
        delimiter=',',
        header='Voltage (mV), Current (uA)',
        comments='',
    )


def build_parser() -> argparse.ArgumentParser:
    """
    Build parser for standalone CV example runner.

    Parameters:
        - None
    Returns:
        - argparse.ArgumentParser: Configured parser.
    """
    parser = argparse.ArgumentParser(description='Run CV with explicit per-technique parameters.')
    parser.add_argument('--device-name', default='Biocoin', help='BLE device name to connect to.')
    parser.add_argument('--output', type=Path, default=Path('./results/CV_output.csv'), help='Output CSV path.')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose console logging.')
    parser.add_argument(
        '--processing-interval',
        type=float,
        default=DEFAULT_PROCESSING_INTERVAL,
        help='Time between processing interrupts (s).',
    )
    parser.add_argument('--max-current', type=float, default=DEFAULT_MAX_CURRENT, help='Maximum current (uA).')
    parser.add_argument('--e-start', type=float, default=DEFAULT_E_START, help='Start potential (mV).')
    parser.add_argument('--e-vertex1', type=float, default=DEFAULT_E_VERTEX1, help='First vertex potential (mV).')
    parser.add_argument('--e-vertex2', type=float, default=DEFAULT_E_VERTEX2, help='Second vertex potential (mV).')
    parser.add_argument('--e-step', type=float, default=DEFAULT_E_STEP, help='Potential step size (mV).')
    parser.add_argument('--pulse-width', type=float, default=DEFAULT_PULSE_WIDTH, help='Pulse width (ms).')
    parser.add_argument('--channel', type=int, default=DEFAULT_CHANNEL, help='Electrode channel index (0-3).')
    return parser


async def run(args: argparse.Namespace) -> None:
    """
    Execute CV technique with provided CLI arguments.

    Parameters:
        - args (argparse.Namespace): Parsed CLI arguments.
    Returns:
        - None
    """
    setup_logging(verbose=args.verbose)
    logger.info('Starting standalone CV run.')

    device = BiocoinDevice()
    try:
        await device.connect(name=args.device_name)
        battery_level = await device.get_battery_level()
        logger.info(f'Battery level: {battery_level}%')

        cv = CyclicVoltammetry(device)
        await cv.configure(
            processing_interval=args.processing_interval,
            max_current=args.max_current,
            E_start=args.e_start,
            E_vertex1=args.e_vertex1,
            E_vertex2=args.e_vertex2,
            E_step=args.e_step,
            pulse_width=args.pulse_width,
            channel=args.channel,
        )
        data = await cv.run()
        write_csv(data, args.output)
        logger.info(f'Saved {len(data)} CV points to {args.output}')
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
    Parse CLI arguments and execute the async CV run.

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
        logger.exception('CV run failed.')
        raise SystemExit(1) from exc


if __name__ == '__main__':
    main()
