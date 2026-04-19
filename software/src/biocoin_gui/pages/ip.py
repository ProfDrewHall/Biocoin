import asyncio
import datetime

import pandas as pd
import plotly.express as px
import streamlit as st

from biocoin.techniques.iontophoresis import Iontophoresis
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
    start_background_run,
    trigger_run_poll,
)

PAGE = PAGE_METADATA['ip']
CURRENT_LABEL = 'Stim Current (µA)'

st.set_page_config(page_title=PAGE.page_title, page_icon=PAGE.icon, layout='wide')
st.title(PAGE.title)
st.caption(PAGE.caption)
require_backend_runtime()
require_device_connection()

initialize_experiment_state('ionto')


async def run_ionto_experiment(
    controller: BackgroundTechniqueRun,
    device,
    *,
    current_monitor_interval: float,
    stim_current: float,
    safety_threshold: float,
    duration: int,
) -> pd.DataFrame:
    ionto = Iontophoresis(device)
    await ionto.configure(
        current_monitor_interval=current_monitor_interval,
        stim_current=stim_current,
        current_safety_threshold=safety_threshold,
    )

    controller.bind_abort(asyncio.get_running_loop(), lambda: device.write_ctrl_command(ionto.Command.STOP))

    await ionto.clear_queue()
    await device.write_ctrl_command(ionto.Command.START)

    loop = asyncio.get_running_loop()
    start_time = loop.time()
    try:
        while True:
            elapsed = min(loop.time() - start_time, duration)
            live_df = pd.DataFrame([[0.0, stim_current], [elapsed, stim_current]], columns=['Time (s)', CURRENT_LABEL])
            controller.set_live_data(live_df)
            controller.set_progress(min(elapsed / duration, 1.0), f'Running... {elapsed:.1f} s / {duration} s')

            if controller.should_stop():
                raise TechniqueAbortRequested
            if elapsed >= duration:
                break
            await asyncio.sleep(min(0.25, duration - elapsed))
    finally:
        await device.write_ctrl_command(ionto.Command.STOP)

    if controller.should_stop():
        raise TechniqueAbortRequested

    return pd.DataFrame([[0.0, stim_current], [duration, stim_current]], columns=['Time (s)', CURRENT_LABEL])


def finalize_ionto_run() -> BackgroundTechniqueRun | None:
    return finalize_experiment_run(
        'ionto',
        aborted_message='Iontophoresis experiment aborted.',
    )


def start_ionto_run(
    device,
    *,
    current_monitor_interval: float,
    stim_current: float,
    safety_threshold: float,
    duration: int,
    exp_name: str,
) -> None:
    st.session_state.ionto_pending_experiment = {
        'name': exp_name,
        'meta': {
            'Date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'Stimulation Current (µA)': stim_current,
            'Safety Threshold (µA)': safety_threshold,
            'Monitor Interval (s)': current_monitor_interval,
            'Duration (s)': duration,
            'Notes': '',
        },
    }
    st.session_state.ionto_message = None
    st.session_state.ionto_run = start_background_run(
        lambda controller: run_ionto_experiment(
            controller,
            device,
            current_monitor_interval=current_monitor_interval,
            stim_current=stim_current,
            safety_threshold=safety_threshold,
            duration=duration,
        )
    )


ionto_run = finalize_ionto_run()
ionto_snapshot = ionto_run.snapshot() if ionto_run is not None else None

cols = st.columns([1, 2], gap='small')

with cols[0].container(border=True):
    st.subheader('Experiment Settings')
    current_monitor_interval = st.number_input('Current Monitor Interval (s)', 0.1, value=1.0)
    stim_current = st.number_input('Stimulation Current (µA)', 1.0, 3000.0, value=100.0)
    safety_threshold = st.number_input('Safety Threshold (µA)', 1.0, 3000.0, value=500.0)
    duration = st.number_input('Duration (s)', 1, 3600, value=60)

    exp_name_input = st.text_input(
        'Experiment Name (optional)',
        value=f'IP Experiment {len(st.session_state.ionto_experiments) + 1}',
        disabled=ionto_snapshot is not None and ionto_snapshot.is_running,
    )

    action_label = 'Abort Experiment' if ionto_snapshot is not None and ionto_snapshot.is_running else 'Run Experiment'
    action_clicked = st.button(action_label, width='stretch', disabled=not st.session_state.connected)

    if action_clicked:
        if ionto_snapshot is not None and ionto_snapshot.is_running:
            ionto_run.request_abort()
        else:
            exp_name = exp_name_input.strip() or f'IP Experiment {len(st.session_state.ionto_experiments) + 1}'
            start_ionto_run(
                st.session_state.biocoin_device,
                current_monitor_interval=current_monitor_interval,
                stim_current=stim_current,
                safety_threshold=safety_threshold,
                duration=duration,
                exp_name=exp_name,
            )
        st.rerun()


with cols[1].container(border=True):
    st.subheader('Results')

    if st.session_state.ionto_message is not None:
        level, message = st.session_state.ionto_message
        getattr(st, level)(message)

    if ionto_snapshot is not None and ionto_snapshot.is_running:
        if ionto_snapshot.abort_requested:
            st.warning(ionto_snapshot.status_text)
        else:
            st.info(ionto_snapshot.status_text)
        st.progress(ionto_snapshot.progress, text=ionto_snapshot.status_text)
        live_df = ionto_snapshot.live_data
        if isinstance(live_df, pd.DataFrame) and not live_df.empty:
            fig = px.line(live_df, x='Time (s)', y=CURRENT_LABEL, title='Live Iontophoresis Protocol')
            st.plotly_chart(fig, width='stretch', key='ionto_live_plot')

    if st.session_state.ionto_experiments:
        exp_names = list(st.session_state.ionto_experiments.keys())
        selected_exps = st.multiselect('Select experiments', exp_names, default=exp_names)
        display_mode = st.radio('Display mode', ['Plot', 'Table'], horizontal=True)

        if display_mode == 'Plot':
            fig = px.line(title='Iontophoresis Results')
            for exp_name in selected_exps:
                df = st.session_state.ionto_experiments[exp_name].data
                fig.add_scatter(x=df['Time (s)'], y=df[CURRENT_LABEL], mode='lines', name=exp_name)
            st.plotly_chart(fig, width='stretch', key='ionto_results_plot')
        else:
            combined_df = pd.concat(
                [
                    record.data.assign(Experiment=name)
                    for name, record in st.session_state.ionto_experiments.items()
                    if name in selected_exps
                ],
                ignore_index=True,
            )
            st.dataframe(combined_df, width='stretch')

        st.divider()
        render_notes_editor(st.session_state.ionto_experiments, key_prefix='ionto')

        st.divider()
        st.subheader('Download Data')
        render_experiment_downloads(
            st.session_state.ionto_experiments,
            selected_exps,
            csv_filename='IP_results.csv',
            excel_filename='IP_results.xlsx',
            key_prefix='ionto',
        )


if ionto_snapshot is not None and ionto_snapshot.is_running:
    trigger_run_poll()


