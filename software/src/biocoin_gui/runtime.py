import asyncio
import csv
import sys
import threading
import time
from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass, field
from io import BytesIO, StringIO
from typing import Any

import pandas as pd
import streamlit as st


@dataclass(frozen=True, slots=True)
class PageMetadata:
    """
    Display metadata used to configure a GUI page consistently.
    """

    page_title: str
    title: str
    caption: str
    icon: str


@dataclass(slots=True)
class ExperimentRecord:
    """
    Saved experiment payload containing tabular data and metadata.
    """

    data: pd.DataFrame
    meta: dict[str, Any]


PAGE_METADATA: dict[str, PageMetadata] = {
    'ca': PageMetadata(
        page_title='Chronoamperometry',
        title='Chronoamperometry (CA)',
        caption='Configure and run Chronoamperometry experiments.',
        icon=':material/monitoring:',
    ),
    'cv': PageMetadata(
        page_title='Cyclic Voltammetry',
        title='Cyclic Voltammetry (CV)',
        caption='Configure and run cyclic voltammetry experiments.',
        icon=':material/autorenew:',
    ),
    'dpv': PageMetadata(
        page_title='Differential Pulse Voltammetry',
        title='Differential Pulse Voltammetry (DPV)',
        caption='Configure and run differential pulse voltammetry experiments.',
        icon=':material/timeline:',
    ),
    'imp': PageMetadata(
        page_title='Impedance',
        title='Impedance (IMP)',
        caption='Measure impedance magnitude and phase.',
        icon=':material/ssid_chart:',
    ),
    'ip': PageMetadata(
        page_title='Iontophoresis',
        title='Iontophoresis (IP)',
        caption='Configure and run iontophoresis protocols.',
        icon=':material/bolt:',
    ),
    'ocp': PageMetadata(
        page_title='Open Circuit Potential',
        title='Open Circuit Potential (OCP)',
        caption='Measure open-circuit potential as a function of time.',
        icon=':material/show_chart:',
    ),
    'swv': PageMetadata(
        page_title='Square Wave Voltammetry',
        title='Square Wave Voltammetry (SWV)',
        caption='Configure and run square wave voltammetry experiments.',
        icon=':material/waves:',
    ),
    'temp': PageMetadata(
        page_title='Temperature',
        title='Temperature Monitoring (TEMP)',
        caption='Measure temperature.',
        icon=':material/device_thermostat:',
    ),
}


def require_backend_runtime() -> None:
    """
    Stop the app if the current Python runtime is too old for the backend package.

    Parameters:
        - None
    Returns:
        - None
    Raises:
        - BaseException: Streamlit stop signal when the active Python runtime is older than 3.13.
    """

    if sys.version_info >= (3, 13):  # noqa: UP036
        return

    current = f'{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}'
    st.error(f'This Biocoin backend uses Python 3.13 syntax. The GUI is currently running on Python {current}.')
    st.write('Restart Streamlit with the project environment so the backend can be imported.')
    st.code('uv run --python 3.13 streamlit run src/biocoin_gui/app.py', language='bash')
    st.stop()


def ensure_session_state(defaults: Mapping[str, Any]) -> None:
    """
    Initialize missing ``st.session_state`` keys from the provided defaults.

    Parameters:
        - defaults (Mapping[str, Any]): Default values to assign for missing session-state keys.
    Returns:
        - None
    """

    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value


def initialize_experiment_state(prefix: str) -> None:
    """
    Initialize standard session-state fields for a technique page.

    Parameters:
        - prefix (str): Session-state prefix for the technique.
    Returns:
        - None
    """

    ensure_session_state(
        {
            f'{prefix}_experiments': {},
            f'{prefix}_run': None,
            f'{prefix}_pending_experiment': None,
            f'{prefix}_message': None,
        }
    )


def finalize_experiment_run(
    prefix: str,
    *,
    aborted_message: str,
    record_builder: Callable[[Any, dict[str, Any]], ExperimentRecord] | None = None,
) -> 'BackgroundTechniqueRun | None':
    """
    Finalize a background run and persist its result into session state when complete.

    Parameters:
        - prefix (str): Session-state prefix for the technique.
        - aborted_message (str): Message shown when the run is aborted by the user.
        - record_builder (Callable[[Any, dict[str, Any]], ExperimentRecord] | None): Optional builder used to
          convert the raw result and metadata into an ``ExperimentRecord``.
    Returns:
        - BackgroundTechniqueRun | None: The active controller if the run is still in progress, otherwise ``None``.
    Raises:
        - KeyError: If the expected technique session-state keys have not been initialized.
        - Exception: Propagated from ``record_builder`` when result persistence fails.
    """

    controller = st.session_state[f'{prefix}_run']
    if controller is None:
        return None

    snapshot = controller.snapshot()
    if not snapshot.is_finished:
        return controller

    pending = st.session_state[f'{prefix}_pending_experiment']
    if snapshot.error:
        st.session_state[f'{prefix}_message'] = ('error', snapshot.error)
    elif snapshot.aborted:
        st.session_state[f'{prefix}_message'] = ('warning', aborted_message)
    elif pending is not None and snapshot.result is not None:
        builder = record_builder or (lambda result, meta: ExperimentRecord(data=result, meta=meta))
        record = builder(snapshot.result, dict(pending['meta']))
        st.session_state[f'{prefix}_experiments'][pending['name']] = record
        st.session_state[f'{prefix}_message'] = ('success', 'Experiment completed successfully!')

    st.session_state[f'{prefix}_run'] = None
    st.session_state[f'{prefix}_pending_experiment'] = None
    return None


def render_notes_editor(
    records: Mapping[str, ExperimentRecord],
    *,
    key_prefix: str,
    height: int = 150,
) -> None:
    """
    Render a notes editor for the selected saved experiment.

    Parameters:
        - records (Mapping[str, ExperimentRecord]): Saved experiment records keyed by experiment name.
        - key_prefix (str): Prefix used to build stable Streamlit widget keys.
        - height (int): Height of the notes text area in pixels.
    Returns:
        - None
    """

    exp_names = list(records.keys())
    note_exp = st.selectbox('Edit notes', exp_names, key=f'{key_prefix}_note_exp')
    note = st.text_area(
        'Notes',
        records[note_exp].meta.get('Notes', ''),
        height=height,
        key=f'{key_prefix}_notes',
    )
    if st.button('Save Notes', width='stretch', key=f'{key_prefix}_save_notes'):
        records[note_exp].meta['Notes'] = note
        st.success('Notes saved.')


def render_experiment_downloads(
    records: Mapping[str, ExperimentRecord],
    selected_names: list[str],
    *,
    csv_filename: str,
    excel_filename: str,
    key_prefix: str,
) -> None:
    """
    Render CSV and Excel download buttons for the selected experiments.

    Parameters:
        - records (Mapping[str, ExperimentRecord]): Saved experiment records keyed by experiment name.
        - selected_names (list[str]): Experiment names to include in the export.
        - csv_filename (str): Filename used for the CSV download.
        - excel_filename (str): Filename used for the Excel download.
        - key_prefix (str): Prefix used to build stable Streamlit widget keys.
    Returns:
        - None
    Raises:
        - KeyError: If ``selected_names`` contains an experiment not present in ``records``.
        - Exception: Propagated when CSV or Excel export generation fails.
    """

    if not selected_names:
        return

    selected_records = [(name, records[name]) for name in selected_names]

    csv_buffer = StringIO()
    for index, (name, record) in enumerate(selected_records):
        if index > 0:
            csv_buffer.write('\n\n')
        csv_buffer.write(f'# Experiment: {name}\n')
        csv_writer = csv.writer(csv_buffer)
        for key, value in record.meta.items():
            csv_writer.writerow([f'# {key}: {value}'])
        csv_buffer.write('\n')
        csv_buffer.write(record.data.to_csv(index=False))

    excel_buffer = BytesIO()
    with pd.ExcelWriter(excel_buffer, engine='xlsxwriter') as writer:
        for name, record in selected_records:
            sheet = name[:31]
            meta_df = pd.DataFrame(record.meta.items(), columns=['Parameter', 'Value'])
            meta_df.to_excel(writer, sheet_name=sheet, index=False)
            record.data.to_excel(writer, sheet_name=sheet, startrow=len(meta_df) + 2, index=False)
            worksheet = writer.sheets[sheet]
            for column_index in range(len(meta_df.columns) + len(record.data.columns)):
                worksheet.set_column(column_index, column_index, 20)

    col1, col2 = st.columns(2)
    with col1:
        st.download_button(
            'Download CSV',
            csv_buffer.getvalue().encode('utf-8'),
            csv_filename,
            'text/csv',
            width='stretch',
            key=f'{key_prefix}_download_csv',
        )
    with col2:
        st.download_button(
            'Download Excel',
            excel_buffer.getvalue(),
            excel_filename,
            'application/vnd.openxmlformats-officedocument.spreadsheetml.sheet',
            width='stretch',
            key=f'{key_prefix}_download_excel',
        )


def require_device_connection() -> None:
    """
    Stop the page unless there is an active connected Biocoin device in session state.

    Parameters:
        - None
    Returns:
        - None
    Raises:
        - BaseException: Streamlit stop signal when there is no active device connection.
    """

    ensure_session_state(
        {
            'connected': False,
            'biocoin_device': None,
        }
    )

    device = st.session_state.biocoin_device
    is_connected = bool(
        st.session_state.connected
        and device is not None
        and getattr(getattr(device, 'client', None), 'is_connected', False)
    )

    if not is_connected:
        st.session_state.connected = False
        st.session_state.biocoin_device = None
        st.warning('Please connect to a Biocoin device from the sidebar before using this page.')
        st.stop()


class TechniqueAbortRequested(Exception):
    """
    Raised when a running technique is explicitly aborted by the user.
    """


@dataclass(slots=True)
class BackgroundRunSnapshot:
    """
    Immutable view of the current background-run state for UI rendering.
    """

    progress: float
    status_text: str
    live_data: Any
    result: Any
    error: str | None
    is_running: bool
    is_finished: bool
    abort_requested: bool
    aborted: bool


@dataclass(slots=True)
class BackgroundTechniqueRun:
    """
    Thread-safe shared state for a background technique run.
    """

    _lock: threading.Lock = field(default_factory=threading.Lock)
    _stop_requested: threading.Event = field(default_factory=threading.Event)
    _abort_sent: threading.Event = field(default_factory=threading.Event)
    _loop: asyncio.AbstractEventLoop | None = None
    _abort_coro_factory: Callable[[], Awaitable[Any]] | None = None
    progress: float = 0.0
    status_text: str = 'Starting...'
    live_data: Any = None
    result: Any = None
    error: str | None = None
    is_running: bool = True
    is_finished: bool = False
    aborted: bool = False
    worker: threading.Thread | None = None

    def snapshot(self) -> BackgroundRunSnapshot:
        """
        Return a thread-safe snapshot of the current background-run state.

        Parameters:
            - None
        Returns:
            - BackgroundRunSnapshot: Immutable copy of the run state for UI rendering.
        """

        with self._lock:
            return BackgroundRunSnapshot(
                progress=self.progress,
                status_text=self.status_text,
                live_data=self.live_data,
                result=self.result,
                error=self.error,
                is_running=self.is_running,
                is_finished=self.is_finished,
                abort_requested=self._stop_requested.is_set(),
                aborted=self.aborted,
            )

    def set_progress(self, progress: float, status_text: str) -> None:
        """
        Update the run progress fraction and status text.

        Parameters:
            - progress (float): Normalized progress fraction in the range ``[0, 1]``.
            - status_text (str): User-facing status message.
        Returns:
            - None
        """

        with self._lock:
            self.progress = max(0.0, min(progress, 1.0))
            self.status_text = status_text

    def set_live_data(self, live_data: Any) -> None:
        """
        Replace the latest live data payload for the active run.

        Parameters:
            - live_data (Any): Latest live data object to expose to the UI.
        Returns:
            - None
        """

        with self._lock:
            self.live_data = live_data

    def bind_abort(self, loop: asyncio.AbstractEventLoop, abort_coro_factory: Callable[[], Awaitable[Any]]) -> None:
        """
        Register the event loop and coroutine used to send an abort command.

        Parameters:
            - loop (asyncio.AbstractEventLoop): Event loop that owns the device connection.
            - abort_coro_factory (Callable[[], Awaitable[Any]]): Factory returning the coroutine that sends the
              technique stop command.
        Returns:
            - None
        """

        with self._lock:
            self._loop = loop
            self._abort_coro_factory = abort_coro_factory

    def request_abort(self) -> None:
        """
        Mark the run for stopping and attempt to send the device abort coroutine once.

        Parameters:
            - None
        Returns:
            - None
        """

        self._stop_requested.set()
        with self._lock:
            self.status_text = 'Stopping experiment...'
            loop = self._loop
            abort_coro_factory = self._abort_coro_factory

        if loop is None or abort_coro_factory is None or self._abort_sent.is_set():
            return

        self._abort_sent.set()
        try:
            future = asyncio.run_coroutine_threadsafe(abort_coro_factory(), loop)
            future.add_done_callback(lambda _: None)
        except Exception:
            self._abort_sent.clear()

    def should_stop(self) -> bool:
        """
        Report whether the run has been asked to stop.

        Parameters:
            - None
        Returns:
            - bool: ``True`` when an abort has been requested.
        """

        return self._stop_requested.is_set()

    def finish(self, *, result: Any = None, error: str | None = None, aborted: bool = False) -> None:
        """
        Mark the run as finished and record its terminal outcome.

        Parameters:
            - result (Any): Final successful result payload.
            - error (str | None): Error message to expose when the run fails.
            - aborted (bool): Whether the run was explicitly aborted.
        Returns:
            - None
        """

        with self._lock:
            self.result = result
            self.error = error
            self.aborted = aborted
            self.is_running = False
            self.is_finished = True
            if error is not None:
                self.status_text = error
            elif aborted:
                self.status_text = 'Experiment aborted.'
                self.progress = 0.0
            else:
                self.status_text = 'Experiment complete.'
                self.progress = 1.0


def start_background_run(worker_factory: Callable[[BackgroundTechniqueRun], Awaitable[Any]]) -> BackgroundTechniqueRun:
    """
    Start an async worker in a dedicated thread and return its shared run controller.

    Parameters:
        - worker_factory (Callable[[BackgroundTechniqueRun], Awaitable[Any]]): Coroutine factory that runs the
          technique using the shared controller.
    Returns:
        - BackgroundTechniqueRun: Shared controller used by the page to observe and abort the worker.
    """

    controller = BackgroundTechniqueRun()

    def runner() -> None:
        """
        Execute the background worker coroutine on its dedicated event loop.

        Parameters:
            - None
        Returns:
            - None
        """

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            result = loop.run_until_complete(worker_factory(controller))
        except TechniqueAbortRequested:
            controller.finish(aborted=True)
        except Exception as exc:
            controller.finish(error=str(exc))
        else:
            if controller.should_stop():
                controller.finish(aborted=True)
            else:
                controller.finish(result=result)
        finally:
            loop.run_until_complete(loop.shutdown_asyncgens())
            loop.close()

    worker = threading.Thread(target=runner, daemon=True)
    controller.worker = worker
    worker.start()
    return controller


def trigger_run_poll(interval_s: float = 0.5) -> None:
    """
    Rerun the Streamlit page after a short delay while a background run is active.

    Parameters:
        - interval_s (float): Delay in seconds before rerunning the page.
    Returns:
        - None
    Raises:
        - BaseException: Streamlit rerun signal after the delay elapses.
    """

    time.sleep(interval_s)
    st.rerun()


async def run_fixed_duration_live(
    technique,
    device,
    duration,
    *,
    poll_interval,
    on_samples,
    on_progress,
    flush_delay=1.0,
    should_stop=None,
) -> None:
    """
    Run a fixed-duration technique while streaming intermediate samples to the UI.

    Parameters:
        - technique: Configured technique instance with ``start`` and ``stop`` methods.
        - device: Connected Biocoin device used to send control commands.
        - duration: Requested experiment duration in seconds.
        - poll_interval: Delay between queue drains while the technique is running.
        - on_samples: Callback receiving drained samples after each polling interval.
        - on_progress: Callback receiving elapsed and total duration values.
        - flush_delay: Optional delay after sending ``STOP`` before draining final samples.
        - should_stop: Optional callback that returns ``True`` when the run should abort.
    Returns:
        - None
    Raises:
        - TechniqueAbortRequested: If the caller requests aborting the active run.
        - Exception: Propagated from the technique or device operations during execution.
    """

    await technique.start()
    loop = asyncio.get_running_loop()
    start_time = loop.time()
    device_started = False

    def abort_requested() -> bool:
        """
        Report whether the caller has requested that the active run stop.

        Parameters:
            - None
        Returns:
            - bool: ``True`` when the optional stop callback requests aborting.
        """

        return should_stop is not None and should_stop()

    try:
        await device.write_ctrl_command(technique.Command.START)
        device_started = True

        while True:
            if abort_requested():
                raise TechniqueAbortRequested

            elapsed = loop.time() - start_time
            if elapsed >= duration:
                break

            await asyncio.sleep(min(poll_interval, duration - elapsed))
            on_samples(technique._drain_queue())
            elapsed = min(loop.time() - start_time, duration)
            on_progress(elapsed, duration)

            if abort_requested():
                raise TechniqueAbortRequested
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
    should_stop=None,
) -> None:
    """
    Run a streaming technique until the device reports completion or a completion callback succeeds.

    Parameters:
        - technique: Configured technique instance with ``start``, ``stop``, and ``is_done`` methods.
        - device: Connected Biocoin device used to send control commands.
        - duration: Expected experiment duration in seconds.
        - poll_interval: Delay between queue drains while the technique is running.
        - on_samples: Callback receiving drained samples after each polling interval.
        - on_progress: Callback receiving elapsed and total duration values.
        - is_complete: Optional callback used to determine whether all expected data has been received.
        - max_polls: Maximum number of completion polls after the nominal duration has elapsed.
        - poll_interval_cap: Upper bound for the completion-poll interval.
        - should_stop: Optional callback that returns ``True`` when the run should abort.
    Returns:
        - None
    Raises:
        - TechniqueAbortRequested: If the caller requests aborting the active run.
        - TimeoutError: If the device never reports completion within the allotted polling window.
        - Exception: Propagated from the technique or device operations during execution.
    """

    await technique.start()
    loop = asyncio.get_running_loop()
    start_time = loop.time()
    device_started = False

    def abort_requested() -> bool:
        """
        Report whether the caller has requested that the active run stop.

        Parameters:
            - None
        Returns:
            - bool: ``True`` when the optional stop callback requests aborting.
        """

        return should_stop is not None and should_stop()

    try:
        await device.write_ctrl_command(technique.Command.START)
        device_started = True

        while True:
            if abort_requested():
                raise TechniqueAbortRequested

            elapsed = loop.time() - start_time
            if elapsed >= duration:
                break

            await asyncio.sleep(min(poll_interval, duration - elapsed))
            on_samples(technique._drain_queue())
            if is_complete is not None and is_complete():
                return
            elapsed = min(loop.time() - start_time, duration)
            on_progress(elapsed, duration)

            if abort_requested():
                raise TechniqueAbortRequested

        completion_poll_interval = min(max(0.1 * duration, 0.01), poll_interval_cap)
        for _ in range(max_polls):
            if abort_requested():
                raise TechniqueAbortRequested
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
