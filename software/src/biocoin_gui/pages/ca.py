import asyncio
import datetime

import pandas as pd
import plotly.express as px
import streamlit as st

from biocoin.techniques import ChronoAmperometry
from biocoin_gui.runtime import (
    PAGE_METADATA,
    BackgroundTechniqueRun,
    TechniqueAbortRequested,
    finalize_experiment_run,
    initialize_experiment_state,
    render_experiment_downloads,
    render_notes_editor,
    require_backend_runtime,
    require_device_connection,
    run_fixed_duration_live,
    start_background_run,
    trigger_run_poll,
)

PAGE = PAGE_METADATA['ca']
CURRENT_LABEL = 'Current (µA)'

st.set_page_config(page_title=PAGE.page_title, page_icon=PAGE.icon, layout='wide')
st.title(PAGE.title)
st.caption(PAGE.caption)
require_backend_runtime()
require_device_connection()

initialize_experiment_state('ca')


def build_ca_figure(df: pd.DataFrame, title: str) -> px.line:
    return px.line(df, x='Time (s)', y=CURRENT_LABEL, title=title)


async def run_ca_experiment(
    controller: BackgroundTechniqueRun,
    device,
    *,
    sampling_interval: float,
    processing_interval: float,
    max_current: float,
    pulse_potential: float,
    channel: int,
    duration: int,
) -> pd.DataFrame:
    ca = ChronoAmperometry(device)
    await ca.configure(
        sampling_interval=sampling_interval,
        processing_interval=processing_interval,
        max_current=max_current,
        pulse_potential=pulse_potential,
        channel=channel,
    )

    controller.bind_abort(asyncio.get_running_loop(), lambda: device.write_ctrl_command(ca.Command.STOP))

    df = pd.DataFrame(columns=['Time (s)', CURRENT_LABEL])
    sample_count = 0

    def append_samples(samples) -> None:
        nonlocal df, sample_count
        for current in samples:
            df.loc[len(df)] = [sample_count * sampling_interval, current]
            sample_count += 1
        if not df.empty:
            controller.set_live_data(df.copy())

    def on_progress(elapsed: float, total: float) -> None:
        controller.set_progress(min(elapsed / total, 1.0), f'Running... {elapsed:.1f} s / {total} s')

    await run_fixed_duration_live(
        ca,
        device,
        duration,
        poll_interval=min(0.25, max(sampling_interval / 2.0, 0.05)),
        on_samples=append_samples,
        on_progress=on_progress,
        should_stop=controller.should_stop,
    )
    controller.set_live_data(df.copy())

    if controller.should_stop():
        raise TechniqueAbortRequested

    return df


def finalize_ca_run() -> BackgroundTechniqueRun | None:
    return finalize_experiment_run(
        'ca',
        aborted_message='Chronoamperometry experiment aborted.',
    )


def start_ca_run(
    device,
    *,
    sampling_interval: float,
    processing_interval: float,
    max_current: float,
    pulse_potential: float,
    channel: int,
    duration: int,
    exp_name: str,
) -> None:
    st.session_state.ca_pending_experiment = {
        'name': exp_name,
        'meta': {
            'Date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'Sampling Interval (s)': sampling_interval,
            'Processing Interval (s)': processing_interval,
            'Max Current (µA)': max_current,
            'Pulse Potential (mV)': pulse_potential,
            'Channel': channel,
            'Duration (s)': duration,
            'Notes': '',
        },
    }
    st.session_state.ca_message = None
    st.session_state.ca_run = start_background_run(
        lambda controller: run_ca_experiment(
            controller,
            device,
            sampling_interval=sampling_interval,
            processing_interval=processing_interval,
            max_current=max_current,
            pulse_potential=pulse_potential,
            channel=channel,
            duration=duration,
        )
    )


ca_run = finalize_ca_run()
ca_snapshot = ca_run.snapshot() if ca_run is not None else None

cols = st.columns([1, 2], gap='small')

with cols[0].container(border=True):
    st.subheader('Experiment Settings')
    max_current = st.number_input('Max Current (µA)', min_value=1.0, max_value=10000.0, value=100.0)
    pulse_potential = st.number_input('Pulse Potential (mV)', min_value=-1000.0, max_value=1000.0, value=200.0)
    duration = st.number_input('Duration (s)', min_value=1, max_value=600, value=15)
    channel = st.selectbox('Channel', options=[0, 1, 2, 3])

    with st.expander('Advanced Settings'):
        sampling_interval = st.number_input('Sampling Interval (s)', min_value=0.1, value=1.0)
        processing_interval = st.number_input('Processing Interval (s)', min_value=0.1, value=1.0)

    exp_name_input = st.text_input(
        'Experiment Name (optional)',
        value=f'CA Experiment {len(st.session_state.ca_experiments) + 1}',
        disabled=ca_snapshot is not None and ca_snapshot.is_running,
    )

    action_label = 'Abort Experiment' if ca_snapshot is not None and ca_snapshot.is_running else 'Run Experiment'
    action_clicked = st.button(action_label, width='stretch', disabled=not st.session_state.connected)

    if action_clicked:
        if ca_snapshot is not None and ca_snapshot.is_running:
            ca_run.request_abort()
        else:
            exp_name = exp_name_input.strip() or f'CA Experiment {len(st.session_state.ca_experiments) + 1}'
            start_ca_run(
                st.session_state.biocoin_device,
                sampling_interval=sampling_interval,
                processing_interval=processing_interval,
                max_current=max_current,
                pulse_potential=pulse_potential,
                channel=channel,
                duration=duration,
                exp_name=exp_name,
            )
        st.rerun()


with cols[1].container(border=True):
    st.subheader('Results')

    if st.session_state.ca_message is not None:
        level, message = st.session_state.ca_message
        getattr(st, level)(message)

    if ca_snapshot is not None and ca_snapshot.is_running:
        if ca_snapshot.abort_requested:
            st.warning(ca_snapshot.status_text)
        else:
            st.info(ca_snapshot.status_text)
        st.progress(ca_snapshot.progress, text=ca_snapshot.status_text)

        live_df = ca_snapshot.live_data
        if isinstance(live_df, pd.DataFrame) and not live_df.empty:
            st.plotly_chart(build_ca_figure(live_df, 'Live Chronoamperometry Data'), width='stretch', key='ca_live')

    if st.session_state.ca_experiments:
        exp_names = list(st.session_state.ca_experiments.keys())
        selected_exps = st.multiselect('Select experiments', exp_names, default=exp_names)

        fig = px.line(title='Chronoamperometry Results')
        for name in selected_exps:
            df = st.session_state.ca_experiments[name].data
            fig.add_scatter(x=df['Time (s)'], y=df[CURRENT_LABEL], name=name)

        st.plotly_chart(fig, width='stretch', key='ca_results_plot')

        st.divider()
        render_notes_editor(st.session_state.ca_experiments, key_prefix='ca', height=120)

        st.divider()
        st.subheader('Download Data')
        render_experiment_downloads(
            st.session_state.ca_experiments,
            selected_exps,
            csv_filename='CA_results.csv',
            excel_filename='CA_results.xlsx',
            key_prefix='ca',
        )


if ca_snapshot is not None and ca_snapshot.is_running:
    trigger_run_poll()


