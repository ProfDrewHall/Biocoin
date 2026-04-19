import asyncio
import datetime

import pandas as pd
import plotly.express as px
import streamlit as st

from biocoin.techniques.temp import Temperature
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

PAGE = PAGE_METADATA['temp']
TEMPERATURE_LABEL = 'Temperature (°C)'

st.set_page_config(page_title=PAGE.page_title, page_icon=PAGE.icon, layout='wide')
st.title(PAGE.title)
st.caption(PAGE.caption)
require_backend_runtime()
require_device_connection()

initialize_experiment_state('temp')


async def run_temp_experiment(
    controller: BackgroundTechniqueRun,
    device,
    *,
    sampling_interval: float,
    processing_interval: float,
    scale_factor: float,
    offset_voltage: float,
    channel: int,
    duration: int,
) -> pd.DataFrame:
    temp = Temperature(device)
    await temp.configure(
        sampling_interval=sampling_interval,
        processing_interval=processing_interval,
        channel=channel,
    )

    controller.bind_abort(asyncio.get_running_loop(), lambda: device.write_ctrl_command(temp.Command.STOP))

    df = pd.DataFrame(columns=['Time (s)', TEMPERATURE_LABEL])
    sample_count = 0

    def append_samples(samples) -> None:
        nonlocal df, sample_count
        for value in samples:
            temperature_c = (value - offset_voltage) * scale_factor
            df.loc[len(df)] = [sample_count * sampling_interval, temperature_c]
            sample_count += 1
        if not df.empty:
            controller.set_live_data(df.copy())

    def on_progress(elapsed: float, total: float) -> None:
        controller.set_progress(min(elapsed / total, 1.0), f'Running... {elapsed:.1f} s / {total} s')

    await run_fixed_duration_live(
        temp,
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


def finalize_temp_run() -> BackgroundTechniqueRun | None:
    return finalize_experiment_run(
        'temp',
        aborted_message='Temperature experiment aborted.',
    )


def start_temp_run(
    device,
    *,
    sampling_interval: float,
    processing_interval: float,
    scale_factor: float,
    offset_voltage: float,
    channel: int,
    duration: int,
    exp_name: str,
) -> None:
    st.session_state.temp_pending_experiment = {
        'name': exp_name,
        'meta': {
            'Date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'Sampling Interval (s)': sampling_interval,
            'Processing Interval (s)': processing_interval,
            'Scale Factor (°C/mV)': scale_factor,
            'Offset Voltage (mV)': offset_voltage,
            'Duration (s)': duration,
            'Channel': channel,
            'Notes': '',
        },
    }
    st.session_state.temp_message = None
    st.session_state.temp_run = start_background_run(
        lambda controller: run_temp_experiment(
            controller,
            device,
            sampling_interval=sampling_interval,
            processing_interval=processing_interval,
            scale_factor=scale_factor,
            offset_voltage=offset_voltage,
            channel=channel,
            duration=duration,
        )
    )


temp_run = finalize_temp_run()
temp_snapshot = temp_run.snapshot() if temp_run is not None else None

cols = st.columns([1, 2], gap='small')

with cols[0].container(border=True):
    st.subheader('Experiment Settings')
    duration = st.number_input('Duration (s)', 1, 3600, value=60)
    channel = st.selectbox('Channel', options=[0, 1, 2])
    scale_factor = st.number_input('Scale Factor (°C/mV)', min_value=0.0, value=1.0, step=0.01)
    offset_voltage = st.number_input('Offset Voltage (mV)', value=0.0, step=1.0)

    with st.expander('Advanced Settings'):
        sampling_interval = st.number_input('Sampling Interval (s)', 0.01, value=0.5)
        processing_interval = st.number_input('Processing Interval (s)', 0.01, value=0.5)

    exp_name_input = st.text_input(
        'Experiment Name (optional)',
        value=f'TEMP Experiment {len(st.session_state.temp_experiments) + 1}',
        disabled=temp_snapshot is not None and temp_snapshot.is_running,
    )

    action_label = 'Abort Experiment' if temp_snapshot is not None and temp_snapshot.is_running else 'Run Experiment'
    action_clicked = st.button(action_label, width='stretch', disabled=not st.session_state.connected)

    if action_clicked:
        if temp_snapshot is not None and temp_snapshot.is_running:
            temp_run.request_abort()
        else:
            exp_name = exp_name_input.strip() or f'TEMP Experiment {len(st.session_state.temp_experiments) + 1}'
            start_temp_run(
                st.session_state.biocoin_device,
                sampling_interval=sampling_interval,
                processing_interval=processing_interval,
                scale_factor=scale_factor,
                offset_voltage=offset_voltage,
                channel=channel,
                duration=duration,
                exp_name=exp_name,
            )
        st.rerun()


with cols[1].container(border=True):
    st.subheader('Results')

    if st.session_state.temp_message is not None:
        level, message = st.session_state.temp_message
        getattr(st, level)(message)

    if temp_snapshot is not None and temp_snapshot.is_running:
        if temp_snapshot.abort_requested:
            st.warning(temp_snapshot.status_text)
        else:
            st.info(temp_snapshot.status_text)
        st.progress(temp_snapshot.progress, text=temp_snapshot.status_text)
        live_df = temp_snapshot.live_data
        if isinstance(live_df, pd.DataFrame) and not live_df.empty:
            fig = px.line(live_df, x='Time (s)', y=TEMPERATURE_LABEL, title='Live Temperature Data')
            st.plotly_chart(fig, width='stretch', key='temp_live_plot')

    if st.session_state.temp_experiments:
        exp_names = list(st.session_state.temp_experiments.keys())
        selected_exps = st.multiselect('Select experiments', exp_names, default=exp_names)
        display_mode = st.radio('Display mode', ['Plot', 'Table'], horizontal=True)

        if display_mode == 'Plot':
            fig = px.line(title='Temperature Results')
            for name in selected_exps:
                df = st.session_state.temp_experiments[name].data
                fig.add_scatter(x=df['Time (s)'], y=df[TEMPERATURE_LABEL], mode='lines', name=name)
            st.plotly_chart(fig, width='stretch', key='temp_results_plot')
        else:
            combined_df = pd.concat(
                [
                    record.data.assign(Experiment=name)
                    for name, record in st.session_state.temp_experiments.items()
                    if name in selected_exps
                ],
                ignore_index=True,
            )
            st.dataframe(combined_df, width='stretch')

        st.divider()
        render_notes_editor(st.session_state.temp_experiments, key_prefix='temp')

        st.divider()
        st.subheader('Download Data')
        render_experiment_downloads(
            st.session_state.temp_experiments,
            selected_exps,
            csv_filename='TEMP_results.csv',
            excel_filename='TEMP_results.xlsx',
            key_prefix='temp',
        )


if temp_snapshot is not None and temp_snapshot.is_running:
    trigger_run_poll()


