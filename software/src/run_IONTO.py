"""
Standalone Iontophoresis example CLI with explicit parameters.
"""

import argparse
import asyncio

from biocoin.device import BiocoinDevice
from biocoin.errors import BiocoinError, ConfigError, DeviceStatusError, TransportError
from biocoin.techniques import Iontophoresis
from utils.logging_util import get_logger, setup_logging

logger = get_logger(__name__)
DEFAULT_DURATION = 150
DEFAULT_CURRENT_MONITOR_INTERVAL = 1.0
DEFAULT_STIM_CURRENT = 5.0
DEFAULT_CURRENT_SAFETY_THRESHOLD = 20.0
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


def build_parser() -> argparse.ArgumentParser:
    """
    Build parser for standalone Iontophoresis example runner.

    Parameters:
        - None
    Returns:
        - argparse.ArgumentParser: Configured parser.
    """
    parser = argparse.ArgumentParser(description='Run Iontophoresis with explicit per-technique parameters.')
    parser.add_argument('--device-name', default='Biocoin', help='BLE device name to connect to.')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose console logging.')
    parser.add_argument(
        '--duration',
        type=int,
        default=DEFAULT_DURATION,
        help='Run duration (s).',
    )
    parser.add_argument(
        '--current-monitor-interval',
        type=float,
        default=DEFAULT_CURRENT_MONITOR_INTERVAL,
        help='Monitor interval (s).',
    )
    parser.add_argument(
        '--stim-current',
        type=float,
        default=DEFAULT_STIM_CURRENT,
        help='Stim current (uA).',
    )
    parser.add_argument(
        '--current-safety-threshold',
        type=float,
        default=DEFAULT_CURRENT_SAFETY_THRESHOLD,
        help='Safety threshold current (uA).',
    )
    return parser


async def run(args: argparse.Namespace) -> None:
    """
    Execute Iontophoresis technique with provided CLI arguments.

    Parameters:
        - args (argparse.Namespace): Parsed CLI arguments.
    Returns:
        - None
    """
    setup_logging(verbose=args.verbose)
    logger.info('Starting standalone Iontophoresis run.')

    device = BiocoinDevice()
    try:
        await device.connect(name=args.device_name)
        battery_level = await device.get_battery_level()
        logger.info(f'Battery level: {battery_level}%')

        ionto = Iontophoresis(device)
        await ionto.configure(
            current_monitor_interval=args.current_monitor_interval,
            stim_current=args.stim_current,
            current_safety_threshold=args.current_safety_threshold,
        )
        data = await ionto.run(duration=args.duration)
        logger.info(f'Iontophoresis run complete ({args.duration}s). Return shape={data.shape}')
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
    Parse CLI arguments and execute the async Iontophoresis run.

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
        logger.exception('Iontophoresis run failed.')
        raise SystemExit(1) from exc


if __name__ == '__main__':
    main()
