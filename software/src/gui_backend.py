import asyncio
import sys

import streamlit as st


def require_backend_runtime() -> None:
    if sys.version_info >= (3, 13):
        return

    current = f'{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}'
    st.error(
        'This Biocoin backend uses Python 3.13 syntax. '
        f'The GUI is currently running on Python {current}.'
    )
    st.write('Restart Streamlit with the project environment so the backend can be imported.')
    st.code('uv run streamlit run src/Home.py', language='bash')
    st.stop()


async def run_fixed_duration_live(
    technique,
    device,
    duration,
    *,
    poll_interval,
    on_samples,
    on_progress,
    flush_delay=1.0,
) -> None:
    await technique.start()
    loop = asyncio.get_running_loop()
    start_time = loop.time()
    device_started = False

    try:
        await device.write_ctrl_command(technique.Command.START)
        device_started = True

        while True:
            elapsed = loop.time() - start_time
            if elapsed >= duration:
                break

            await asyncio.sleep(min(poll_interval, duration - elapsed))
            on_samples(technique._drain_queue())
            elapsed = min(loop.time() - start_time, duration)
            on_progress(elapsed, duration)
    finally:
        if device_started:
            try:
                await device.write_ctrl_command(technique.Command.STOP)
                if flush_delay > 0:
                    await asyncio.sleep(flush_delay)
                on_samples(technique._drain_queue())
            except Exception:
                pass
        await technique.stop()


async def run_until_done_live(
    technique,
    device,
    duration,
    *,
    poll_interval,
    on_samples,
    on_progress,
    is_complete=None,
    max_polls=10,
    poll_interval_cap=2.0,
) -> None:
    await technique.start()
    loop = asyncio.get_running_loop()
    start_time = loop.time()
    device_started = False

    try:
        await device.write_ctrl_command(technique.Command.START)
        device_started = True

        while True:
            elapsed = loop.time() - start_time
            if elapsed >= duration:
                break

            await asyncio.sleep(min(poll_interval, duration - elapsed))
            on_samples(technique._drain_queue())
            if is_complete is not None and is_complete():
                return
            elapsed = min(loop.time() - start_time, duration)
            on_progress(elapsed, duration)

        completion_poll_interval = min(max(0.1 * duration, 0.01), poll_interval_cap)
        for _ in range(max_polls):
            on_samples(technique._drain_queue())
            if is_complete is not None and is_complete():
                return
            if await technique.is_done():
                return
            await asyncio.sleep(completion_poll_interval)
        await device.write_ctrl_command(technique.Command.STOP)
        total_wait = duration + max_polls * completion_poll_interval
        raise TimeoutError(f'Device did not finish after {total_wait:.3f}s')
    finally:
        if device_started:
            on_samples(technique._drain_queue())
            if is_complete is not None and is_complete():
                try:
                    await device.write_ctrl_command(technique.Command.STOP)
                except Exception:
                    pass
        await technique.stop()
