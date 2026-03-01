"""
Standalone SWV example CLI for running square-wave voltammetry with explicit parameters.
"""

import argparse
import asyncio
from pathlib import Path

import numpy as np

from biocoin.device import BiocoinDevice
from biocoin.errors import BiocoinError, ConfigError, DeviceStatusError, TransportError
from biocoin.techniques import SquareWaveVoltammetry
from utils.logging_util import get_logger, setup_logging

logger = get_logger(__name__)
DEFAULT_FREQUENCY = 100.0
DEFAULT_PROCESSING_INTERVAL = (1.0 / DEFAULT_FREQUENCY) / 2.0
DEFAULT_MAX_CURRENT = 100.0
DEFAULT_E_START = -200.0
DEFAULT_E_STOP = 200.0
DEFAULT_E_AMPLITUDE = 100.0
DEFAULT_E_STEP = 50.0
DEFAULT_PULSE_PERIOD = (1.0 / DEFAULT_FREQUENCY) * 1000.0
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
    Write SWV output to CSV.

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
    Build parser for standalone SWV example runner.

    Parameters:
        - None
    Returns:
        - argparse.ArgumentParser: Configured parser.
    """
    parser = argparse.ArgumentParser(description='Run SWV with explicit per-technique parameters.')
    parser.add_argument('--device-name', default='Biocoin', help='BLE device name to connect to.')
    parser.add_argument('--output', type=Path, default=Path('./results/SWV_output.csv'), help='Output CSV path.')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose console logging.')
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
    parser.add_argument('--e-start', type=float, default=DEFAULT_E_START, help='Start potential (mV).')
    parser.add_argument('--e-stop', type=float, default=DEFAULT_E_STOP, help='Stop potential (mV).')
    parser.add_argument(
        '--e-amplitude',
        type=float,
        default=DEFAULT_E_AMPLITUDE,
        help='Pulse amplitude (mV).',
    )
    parser.add_argument('--e-step', type=float, default=DEFAULT_E_STEP, help='Baseline step size (mV).')
    parser.add_argument('--pulse-period', type=float, default=DEFAULT_PULSE_PERIOD, help='Pulse period (ms).')
    parser.add_argument(
        '--channel',
        type=int,
        default=DEFAULT_CHANNEL,
        help='Electrode channel index (0-3).',
    )
    return parser


async def run(args: argparse.Namespace) -> None:
    """
    Execute SWV technique with provided CLI arguments.

    Parameters:
        - args (argparse.Namespace): Parsed CLI arguments.
    Returns:
        - None
    """
    setup_logging(verbose=args.verbose)
    logger.info('Starting standalone SWV run.')

    device = BiocoinDevice()
    try:
        await device.connect(name=args.device_name)
        battery_level = await device.get_battery_level()
        logger.info(f'Battery level: {battery_level}%')

        swv = SquareWaveVoltammetry(device)
        await swv.configure(
            processing_interval=args.processing_interval,
            max_current=args.max_current,
            E_start=args.e_start,
            E_stop=args.e_stop,
            E_amplitude=args.e_amplitude,
            E_step=args.e_step,
            pulse_period=args.pulse_period,
            channel=args.channel,
        )
        data = await swv.run()
        write_csv(data, args.output)
        logger.info(f'Saved {len(data)} SWV points to {args.output}')
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
    Parse CLI arguments and execute the async SWV run.

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
        logger.exception('SWV run failed.')
        raise SystemExit(1) from exc


if __name__ == '__main__':
    main()
