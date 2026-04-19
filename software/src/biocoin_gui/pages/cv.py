import asyncio
import datetime

import pandas as pd
import plotly.express as px
import streamlit as st

from biocoin.techniques.cv import CyclicVoltammetry
from biocoin_gui.runtime import (
    PAGE_METADATA,
    BackgroundTechniqueRun,
    ExperimentRecord,
    TechniqueAbortRequested,
    finalize_experiment_run,
    initialize_experiment_state,
    render_experiment_downloads,
    render_notes_editor,
    require_backend_runtime,
    require_device_connection,
    run_until_done_live,
    start_background_run,
    trigger_run_poll,
)

PAGE = PAGE_METADATA['cv']
CURRENT_LABEL = 'Current (µA)'

st.set_page_config(page_title=PAGE.page_title, page_icon=PAGE.icon, layout='wide')
st.title(PAGE.title)
st.caption(PAGE.caption)
require_backend_runtime()
require_device_connection()

initialize_experiment_state('cv')


async def run_cv_experiment(
    controller: BackgroundTechniqueRun,
    device,
    *,
    processing_interval: float,
    max_current: float,
    E_start: float,
    E_vertex1: float,
    E_vertex2: float,
    E_step: float,
    pulse_width: float,
    channel: int,
) -> tuple[pd.DataFrame, float]:
    cv = CyclicVoltammetry(device)
    await cv.configure(
        processing_interval=processing_interval,
        max_current=max_current,
        E_start=E_start,
        E_vertex1=E_vertex1,
        E_vertex2=E_vertex2,
        E_step=E_step,
        pulse_width=pulse_width,
        channel=channel,
    )

    controller.bind_abort(asyncio.get_running_loop(), lambda: device.write_ctrl_command(cv.Command.STOP))

    df = pd.DataFrame(columns=['Voltage (mV)', CURRENT_LABEL])

    def append_samples(samples) -> None:
        nonlocal df
        for current in samples:
            idx = len(df)
            if cv.V is None or idx >= len(cv.V):
                break
            df.loc[len(df)] = [cv.V[idx], current]
        if not df.empty:
            controller.set_live_data(df.copy())

    def on_progress(elapsed: float, total: float) -> None:
        controller.set_progress(min(elapsed / total, 1.0), f'Running... {elapsed:.1f} s / {total:.1f} s')

    await run_until_done_live(
        cv,
        device,
        cv.duration,
        poll_interval=min(0.25, max(pulse_width / 2000.0, 0.05)),
        on_samples=append_samples,
        on_progress=on_progress,
        should_stop=controller.should_stop,
    )
    controller.set_live_data(df.copy())

    if controller.should_stop():
        raise TechniqueAbortRequested
    if cv.V is not None and len(df) != len(cv.V):
        raise RuntimeError(f'Expected {len(cv.V)} CV points, received {len(df)}')

    return df, cv.duration


def finalize_cv_run() -> BackgroundTechniqueRun | None:
    return finalize_experiment_run(
        'cv',
        aborted_message='Cyclic Voltammetry experiment aborted.',
        record_builder=lambda result, meta: ExperimentRecord(
            data=result[0],
            meta={**meta, 'Duration (s)': round(result[1], 3)},
        ),
    )


def start_cv_run(
    device,
    *,
    processing_interval: float,
    max_current: float,
    E_start: float,
    E_vertex1: float,
    E_vertex2: float,
    E_step: float,
    pulse_width: float,
    channel: int,
    exp_name: str,
) -> None:
    st.session_state.cv_pending_experiment = {
        'name': exp_name,
        'meta': {
            'Date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'Processing Interval (s)': processing_interval,
            'Max Current (µA)': max_current,
            'E Start (mV)': E_start,
            'E Vertex 1 (mV)': E_vertex1,
            'E Vertex 2 (mV)': E_vertex2,
            'E Step (mV)': E_step,
            'Pulse Width (ms)': pulse_width,
            'Channel': channel,
            'Notes': '',
        },
    }
    st.session_state.cv_message = None
    st.session_state.cv_run = start_background_run(
        lambda controller: run_cv_experiment(
            controller,
            device,
            processing_interval=processing_interval,
            max_current=max_current,
            E_start=E_start,
            E_vertex1=E_vertex1,
            E_vertex2=E_vertex2,
            E_step=E_step,
            pulse_width=pulse_width,
            channel=channel,
        )
    )


cv_run = finalize_cv_run()
cv_snapshot = cv_run.snapshot() if cv_run is not None else None

cols = st.columns([1, 2], gap='small')

with cols[0].container(border=True):
    st.subheader('Experiment Settings')
    max_current = st.number_input('Max Current (µA)', min_value=1.0, max_value=3000.0, value=100.0)
    E_start = st.number_input('E Start (mV)', value=0.0)
    E_vertex1 = st.number_input('Vertex 1 (mV)', value=-500.0)
    E_vertex2 = st.number_input('Vertex 2 (mV)', value=500.0)
    E_step = st.number_input('Step Size (mV)', min_value=0.6, value=5.0)
    pulse_width = st.number_input('Pulse Width (ms)', min_value=3.0, value=50.0)
    channel = st.selectbox('Channel', options=[0, 1, 2, 3])

    with st.expander('Advanced Settings'):
        processing_interval = st.number_input('Processing Interval (s)', min_value=0.01, value=0.1, step=0.01)

    exp_name_input = st.text_input(
        'Experiment Name (optional)',
        value=f'CV Experiment {len(st.session_state.cv_experiments) + 1}',
        disabled=cv_snapshot is not None and cv_snapshot.is_running,
    )

    action_label = 'Abort Experiment' if cv_snapshot is not None and cv_snapshot.is_running else 'Run Experiment'
    action_clicked = st.button(action_label, width='stretch', disabled=not st.session_state.connected)

    if action_clicked:
        if cv_snapshot is not None and cv_snapshot.is_running:
            cv_run.request_abort()
        else:
            exp_name = exp_name_input.strip() or f'CV Experiment {len(st.session_state.cv_experiments) + 1}'
            start_cv_run(
                st.session_state.biocoin_device,
                processing_interval=processing_interval,
                max_current=max_current,
                E_start=E_start,
                E_vertex1=E_vertex1,
                E_vertex2=E_vertex2,
                E_step=E_step,
                pulse_width=pulse_width,
                channel=channel,
                exp_name=exp_name,
            )
        st.rerun()


with cols[1].container(border=True):
    st.subheader('Results')

    if st.session_state.cv_message is not None:
        level, message = st.session_state.cv_message
        getattr(st, level)(message)

    if cv_snapshot is not None and cv_snapshot.is_running:
        if cv_snapshot.abort_requested:
            st.warning(cv_snapshot.status_text)
        else:
            st.info(cv_snapshot.status_text)
        st.progress(cv_snapshot.progress, text=cv_snapshot.status_text)
        live_df = cv_snapshot.live_data
        if isinstance(live_df, pd.DataFrame) and not live_df.empty:
            fig = px.line(live_df, x='Voltage (mV)', y=CURRENT_LABEL, title='Live Cyclic Voltammetry')
            st.plotly_chart(fig, width='stretch', key='cv_live_plot')

    if st.session_state.cv_experiments:
        exp_names = list(st.session_state.cv_experiments.keys())
        selected_exps = st.multiselect('Select experiments', exp_names, default=exp_names)
        display_mode = st.radio('Display mode', ['Plot', 'Table'], horizontal=True)

        if display_mode == 'Plot':
            fig = px.line(title='CV Results', labels={'x': 'Voltage (mV)', 'y': CURRENT_LABEL})
            for name in selected_exps:
                df = st.session_state.cv_experiments[name].data
                fig.add_scatter(x=df['Voltage (mV)'], y=df[CURRENT_LABEL], name=name)
            st.plotly_chart(fig, width='stretch', key='cv_results_plot')
        else:
            combined_df = pd.concat(
                [
                    record.data.assign(Experiment=name)
                    for name, record in st.session_state.cv_experiments.items()
                    if name in selected_exps
                ],
                ignore_index=True,
            )
            st.dataframe(combined_df, width='stretch')

        st.divider()
        render_notes_editor(st.session_state.cv_experiments, key_prefix='cv')

        st.divider()
        st.subheader('Download Data')
        render_experiment_downloads(
            st.session_state.cv_experiments,
            selected_exps,
            csv_filename='CV_results.csv',
            excel_filename='CV_results.xlsx',
            key_prefix='cv',
        )


if cv_snapshot is not None and cv_snapshot.is_running:
    trigger_run_poll()


