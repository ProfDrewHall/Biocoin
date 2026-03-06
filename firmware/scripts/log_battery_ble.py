#!/usr/bin/env python3
"""
Log battery level from a BLE device to CSV.

Default behavior:
- Connect by address or device name.
- Read the standard Battery Level characteristic (0x2A19).
- Poll on a fixed interval and append rows to CSV.
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import datetime as dt
from pathlib import Path
import sys
from typing import Optional

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError


BATTERY_LEVEL_CHAR_UUID = "00002A19-0000-1000-8000-00805F9B34FB"
BIOCOIN_SERVICE_UUID = "00001523-1212-EFDE-1523-785FEABC93AA"
DEBUG_BATTERY_MV_CHAR_UUID = "00001526-1212-EFDE-1523-785FEABC93AA"
DEBUG_AFE_BURN_CHAR_UUID = "00001527-1212-EFDE-1523-785FEABC93AA"
DEBUG_BATTERY_CMD_SAMPLE_NOW = 0x01


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="BLE battery logger")
    parser.add_argument("--address", help="BLE device address (preferred on Windows)", default=None)
    parser.add_argument("--name", help="BLE device name to scan for", default="Biocoin")
    parser.add_argument(
        "--service-uuid",
        help="Preferred advertising service UUID filter used before name probing",
        default=BIOCOIN_SERVICE_UUID,
    )
    parser.add_argument("--char-uuid", help="Characteristic UUID to read", default=BATTERY_LEVEL_CHAR_UUID)
    parser.add_argument(
        "--debug-voltage",
        action="store_true",
        help="Also read raw battery voltage in mV from a custom characteristic.",
    )
    parser.add_argument(
        "--debug-voltage-char-uuid",
        default=DEBUG_BATTERY_MV_CHAR_UUID,
        help="UUID for raw battery voltage characteristic (uint16 little-endian mV).",
    )
    parser.add_argument(
        "--afe-burn",
        action="store_true",
        help="Enable AFE burn-load mode while logging to accelerate battery discharge.",
    )
    parser.add_argument(
        "--afe-burn-char-uuid",
        default=DEBUG_AFE_BURN_CHAR_UUID,
        help="UUID for AFE burn-load control characteristic (uint8: 0=off, 1=on).",
    )
    parser.add_argument(
        "--leave-afe-burn-on-exit",
        action="store_true",
        help="Do not disable AFE burn mode when the script exits.",
    )
    parser.add_argument("--interval-sec", type=float, default=120.0, help="Polling interval in seconds")
    parser.add_argument("--output", default="battery_log.csv", help="CSV output path")
    parser.add_argument(
        "--connect-timeout-sec",
        type=float,
        default=20.0,
        help="BLE connect+service-discovery timeout per attempt in seconds.",
    )
    parser.add_argument(
        "--connect-retries",
        type=int,
        default=3,
        help="Number of BLE connect attempts before failing.",
    )
    parser.add_argument(
        "--connect-retry-delay-sec",
        type=float,
        default=2.0,
        help="Delay between BLE connect retries in seconds.",
    )
    parser.add_argument(
        "--no-cached-services",
        action="store_true",
        help="On Windows, request uncached GATT service discovery.",
    )
    parser.add_argument(
        "--duration-min",
        type=float,
        default=0.0,
        help="Optional total runtime in minutes (0 = run until Ctrl+C)",
    )
    parser.add_argument(
        "--print-discovered",
        action="store_true",
        help="Print all discovered devices and advertised service UUIDs during scans.",
    )
    return parser.parse_args()


def _normalize_uuid(value: Optional[str]) -> str:
    if value is None:
        return ""
    return value.strip().lower()


def _extract_service_uuids(device, adv_data) -> list[str]:
    uuids: list[str] = []
    if adv_data is not None:
        advertised = getattr(adv_data, "service_uuids", None)
        if advertised:
            uuids.extend(advertised)

    metadata = getattr(device, "metadata", None) or {}
    meta_uuids = metadata.get("uuids") or metadata.get("service_uuids") or []
    uuids.extend(meta_uuids)

    return [_normalize_uuid(u) for u in uuids if u]


async def _discover_with_advertising(timeout: float):
    try:
        discovered = await BleakScanner.discover(timeout=timeout, return_adv=True)
        return list(discovered.values())
    except TypeError:
        devices = await BleakScanner.discover(timeout=timeout)
        return [(device, None) for device in devices]


def _print_discovered_devices(discovered) -> None:
    print(f"Discovered {len(discovered)} device(s):")
    for d, adv_data in discovered:
        uuids = _extract_service_uuids(d, adv_data)
        uuids_str = ", ".join(uuids) if uuids else "(none)"
        rssi = getattr(adv_data, "rssi", None)
        if rssi is None:
            rssi = getattr(d, "rssi", None)
        rssi_str = f"{rssi} dBm" if rssi is not None else "unknown"
        print(f"  - name={d.name or '(unknown)'} address={d.address} rssi={rssi_str} uuids={uuids_str}")


async def find_device(
    address: Optional[str],
    name: Optional[str],
    service_uuid: Optional[str],
    print_discovered: bool = False,
) -> Optional[str]:
    if address:
        return address

    target_name = (name or "").lower()
    target_service_uuid = _normalize_uuid(service_uuid)
    print(
        "Scanning BLE devices "
        f"(service UUID: {service_uuid or 'disabled'}, name contains: '{name or ''}')..."
    )

    while True:
        discovered = await _discover_with_advertising(timeout=8.0)
        if not discovered:
            print("No BLE devices found in this scan window. Retrying...")
            await asyncio.sleep(2.0)
            continue
        if print_discovered:
            _print_discovered_devices(discovered)

        service_candidates = []
        for d, adv_data in discovered:
            uuids = _extract_service_uuids(d, adv_data)
            if target_service_uuid and target_service_uuid in uuids:
                service_candidates.append(d)

        if service_candidates:
            if target_name:
                for d in service_candidates:
                    if (d.name or "").lower().find(target_name) >= 0:
                        print(f"Found {d.name} with service UUID at {d.address}")
                        return d.address

            # Service UUID match is strong enough if no name requirement.
            selected = service_candidates[0]
            print(f"Found service UUID match at {selected.address} ({selected.name})")
            return selected.address

        if target_name:
            # Fallback to name-only match.
            for d, _ in discovered:
                if (d.name or "").lower().find(target_name) >= 0:
                    print(f"Found {d.name} at {d.address} (name fallback)")
                    return d.address

        print("No matching device found. Retrying...")
        await asyncio.sleep(2.0)


def read_u8(payload: bytes) -> int:
    if len(payload) < 1:
        raise ValueError("Empty battery payload")
    return int(payload[0])


def read_u16_le(payload: bytes) -> int:
    if len(payload) < 2:
        raise ValueError("Battery voltage payload is shorter than 2 bytes")
    return int(payload[0] | (payload[1] << 8))


def _target_address(target) -> str:
    return getattr(target, "address", str(target))


def _make_client_kwargs(args: argparse.Namespace, attempt_index: int) -> dict:
    kwargs: dict = {"timeout": max(5.0, float(args.connect_timeout_sec))}
    # On Windows, retries can switch to uncached services to avoid stale-cache failures.
    if sys.platform == "win32" and (args.no_cached_services or attempt_index > 1):
        kwargs["winrt"] = {"use_cached_services": False}
    return kwargs


def _is_not_connected_error(exc: Exception) -> bool:
    return "not connected" in str(exc).strip().lower()


async def run(args: argparse.Namespace) -> None:
    target = await find_device(
        args.address,
        args.name,
        args.service_uuid,
        print_discovered=args.print_discovered,
    )
    if not target:
        raise RuntimeError("Could not resolve target device")
    address = _target_address(target)

    output_path = Path(args.output)
    write_header = not output_path.exists()

    end_time = None
    if args.duration_min > 0:
        end_time = dt.datetime.now(dt.timezone.utc) + dt.timedelta(minutes=args.duration_min)

    connect_retries = max(1, int(args.connect_retries))
    last_connect_error: Exception | None = None
    for attempt in range(1, connect_retries + 1):
        try:
            client_kwargs = _make_client_kwargs(args, attempt)
            print(f"Connecting to {address} (attempt {attempt}/{connect_retries})...")
            async with BleakClient(target, **client_kwargs) as client:
                print("Connected.")
                debug_uuid = args.debug_voltage_char_uuid.strip()
                afe_burn_uuid = args.afe_burn_char_uuid.strip()
                debug_refresh_supported = True
                if args.debug_voltage and not debug_uuid:
                    print(
                        "Warning: --debug-voltage requested but --debug-voltage-char-uuid is empty; "
                        "skipping voltage reads."
                    )
                if args.afe_burn and not afe_burn_uuid:
                    print("Warning: --afe-burn requested but --afe-burn-char-uuid is empty; skipping AFE burn control.")

                if args.afe_burn and afe_burn_uuid:
                    await client.write_gatt_char(afe_burn_uuid, bytes([1]), response=True)
                    print(f"AFE burn load enabled via {afe_burn_uuid}.")

                try:
                    with output_path.open("a", encoding="utf-8", newline="") as f:
                        writer = csv.writer(f)
                        if write_header:
                            writer.writerow(
                                [
                                    "timestamp_utc",
                                    "device_address",
                                    "battery_char_uuid",
                                    "battery_percent",
                                    "debug_voltage_mV",
                                    "afe_burn_enabled",
                                ]
                            )

                        while True:
                            if end_time is not None and dt.datetime.now(dt.timezone.utc) >= end_time:
                                print("Duration reached; stopping.")
                                break

                            payload = bytes(await client.read_gatt_char(args.char_uuid))
                            percent = read_u8(payload)
                            voltage_mv: str = ""
                            if args.debug_voltage and debug_uuid:
                                try:
                                    if debug_refresh_supported:
                                        try:
                                            await client.write_gatt_char(
                                                debug_uuid,
                                                bytes([DEBUG_BATTERY_CMD_SAMPLE_NOW]),
                                                response=True,
                                            )
                                        except Exception as exc:
                                            debug_refresh_supported = False
                                            print(
                                                "Warning: debug voltage refresh command failed; "
                                                f"falling back to read-only mode ({exc})"
                                            )
                                    raw_v = bytes(await client.read_gatt_char(debug_uuid))
                                    voltage_mv = str(read_u16_le(raw_v))
                                except Exception as exc:
                                    voltage_mv = ""
                                    print(f"Warning: debug voltage read failed: {exc}")
                            now = dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")

                            writer.writerow([now, address, args.char_uuid, percent, voltage_mv, int(args.afe_burn)])
                            f.flush()
                            if voltage_mv:
                                print(f"{now} battery={percent}% voltage={voltage_mv}mV")
                            else:
                                print(f"{now} battery={percent}%")

                            await asyncio.sleep(max(0.1, float(args.interval_sec)))
                finally:
                    if args.afe_burn and afe_burn_uuid and not args.leave_afe_burn_on_exit:
                        if client.is_connected:
                            try:
                                await client.write_gatt_char(afe_burn_uuid, bytes([0]), response=True)
                                print(f"AFE burn load disabled via {afe_burn_uuid}.")
                            except Exception as exc:
                                print(f"Warning: failed to disable AFE burn load: {exc}")
                        else:
                            print("Notice: skipping AFE burn disable write because BLE is already disconnected.")

                return
        except (asyncio.TimeoutError, asyncio.CancelledError, BleakError) as exc:
            last_connect_error = exc
            if attempt >= connect_retries:
                break
            if sys.platform == "win32" and attempt == 1 and not args.no_cached_services:
                print(
                    "Connection timed out during service discovery; retrying with uncached WinRT services "
                    "(equivalent to --no-cached-services)."
                )
            elif _is_not_connected_error(exc):
                print(
                    "BLE link dropped during logging/read; retrying connection "
                    f"(attempt {attempt + 1}/{connect_retries})..."
                )
            else:
                print(f"Connection attempt failed: {exc!r}")
            await asyncio.sleep(max(0.1, float(args.connect_retry_delay_sec)))

    if last_connect_error is not None:
        raise RuntimeError(
            "Unable to connect and enumerate services after "
            f"{connect_retries} attempt(s). Last error: {last_connect_error!r}"
        ) from last_connect_error


def main() -> None:
    args = parse_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("Stopped by user.")
    except Exception as exc:
        print(f"Error: {exc}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
