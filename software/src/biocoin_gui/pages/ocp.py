import asyncio
import datetime

import pandas as pd
import plotly.express as px
import streamlit as st

from biocoin.techniques.ocp import OpenCircuitPotential
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

PAGE = PAGE_METADATA['ocp']
POTENTIAL_LABEL = 'Potential (mV)'

st.set_page_config(page_title=PAGE.page_title, page_icon=PAGE.icon, layout='wide')
st.title(PAGE.title)
st.caption(PAGE.caption)
require_backend_runtime()
require_device_connection()

initialize_experiment_state('ocp')


async def run_ocp_experiment(
    controller: BackgroundTechniqueRun,
    device,
    *,
    sampling_interval: float,
    processing_interval: float,
    channel: int,
    duration: int,
) -> pd.DataFrame:
    ocp = OpenCircuitPotential(device)
    await ocp.configure(
        sampling_interval=sampling_interval,
        processing_interval=processing_interval,
        channel=channel,
    )

    controller.bind_abort(asyncio.get_running_loop(), lambda: device.write_ctrl_command(ocp.Command.STOP))

    df = pd.DataFrame(columns=['Time (s)', POTENTIAL_LABEL])
    sample_count = 0

    def append_samples(samples) -> None:
        nonlocal df, sample_count
        for potential in samples:
            df.loc[len(df)] = [sample_count * sampling_interval, potential]
            sample_count += 1
        if not df.empty:
            controller.set_live_data(df.copy())

    def on_progress(elapsed: float, total: float) -> None:
        controller.set_progress(min(elapsed / total, 1.0), f'Running... {elapsed:.1f} s / {total} s')

    await run_fixed_duration_live(
        ocp,
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


def finalize_ocp_run() -> BackgroundTechniqueRun | None:
    return finalize_experiment_run(
        'ocp',
        aborted_message='Open Circuit Potential experiment aborted.',
    )


def start_ocp_run(
    device,
    *,
    sampling_interval: float,
    processing_interval: float,
    channel: int,
    duration: int,
    exp_name: str,
) -> None:
    st.session_state.ocp_pending_experiment = {
        'name': exp_name,
        'meta': {
            'Date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'Sampling Interval (s)': sampling_interval,
            'Processing Interval (s)': processing_interval,
            'Duration (s)': duration,
            'Channel': channel,
            'Notes': '',
        },
    }
    st.session_state.ocp_message = None
    st.session_state.ocp_run = start_background_run(
        lambda controller: run_ocp_experiment(
            controller,
            device,
            sampling_interval=sampling_interval,
            processing_interval=processing_interval,
            channel=channel,
            duration=duration,
        )
    )


ocp_run = finalize_ocp_run()
ocp_snapshot = ocp_run.snapshot() if ocp_run is not None else None

cols = st.columns([1, 2], gap='small')

with cols[0].container(border=True):
    st.subheader('Experiment Settings')
    duration = st.number_input('Duration (s)', min_value=1, max_value=3600, value=30)
    channel = st.selectbox('Channel', options=[0, 1, 2, 3])

    with st.expander('Advanced Settings'):
        sampling_interval = st.number_input('Sampling Interval (s)', min_value=0.1, value=1.0, step=0.1)
        processing_interval = st.number_input('Processing Interval (s)', min_value=0.1, value=1.0, step=0.1)

    exp_name_input = st.text_input(
        'Experiment Name (optional)',
        value=f'OCP Experiment {len(st.session_state.ocp_experiments) + 1}',
        disabled=ocp_snapshot is not None and ocp_snapshot.is_running,
    )

    action_label = 'Abort Experiment' if ocp_snapshot is not None and ocp_snapshot.is_running else 'Run Experiment'
    action_clicked = st.button(action_label, width='stretch', disabled=not st.session_state.connected)

    if action_clicked:
        if ocp_snapshot is not None and ocp_snapshot.is_running:
            ocp_run.request_abort()
        else:
            exp_name = exp_name_input.strip() or f'OCP Experiment {len(st.session_state.ocp_experiments) + 1}'
            start_ocp_run(
                st.session_state.biocoin_device,
                sampling_interval=sampling_interval,
                processing_interval=processing_interval,
                channel=channel,
                duration=duration,
                exp_name=exp_name,
            )
        st.rerun()


with cols[1].container(border=True):
    st.subheader('Results')

    if st.session_state.ocp_message is not None:
        level, message = st.session_state.ocp_message
        getattr(st, level)(message)

    if ocp_snapshot is not None and ocp_snapshot.is_running:
        if ocp_snapshot.abort_requested:
            st.warning(ocp_snapshot.status_text)
        else:
            st.info(ocp_snapshot.status_text)
        st.progress(ocp_snapshot.progress, text=ocp_snapshot.status_text)
        live_df = ocp_snapshot.live_data
        if isinstance(live_df, pd.DataFrame) and not live_df.empty:
            fig = px.line(live_df, x='Time (s)', y=POTENTIAL_LABEL, title='Live OCP Data')
            st.plotly_chart(fig, width='stretch', key='ocp_live_plot')

    if st.session_state.ocp_experiments:
        exp_names = list(st.session_state.ocp_experiments.keys())
        selected_exps = st.multiselect('Select experiments', exp_names, default=exp_names)
        display_mode = st.radio('Display mode', ['Plot', 'Table'], horizontal=True)

        if display_mode == 'Plot':
            fig = px.line(title='OCP Results', labels={'x': 'Time (s)', 'y': POTENTIAL_LABEL})
            for name in selected_exps:
                df = st.session_state.ocp_experiments[name].data
                fig.add_scatter(x=df['Time (s)'], y=df[POTENTIAL_LABEL], name=name)
            st.plotly_chart(fig, width='stretch', key='ocp_results_plot')
        else:
            combined_df = pd.concat(
                [
                    record.data.assign(Experiment=name)
                    for name, record in st.session_state.ocp_experiments.items()
                    if name in selected_exps
                ],
                ignore_index=True,
            )
            st.dataframe(combined_df, width='stretch')

        st.divider()
        render_notes_editor(st.session_state.ocp_experiments, key_prefix='ocp')

        st.divider()
        st.subheader('Download Data')
        render_experiment_downloads(
            st.session_state.ocp_experiments,
            selected_exps,
            csv_filename='OCP_results.csv',
            excel_filename='OCP_results.xlsx',
            key_prefix='ocp',
        )


if ocp_snapshot is not None and ocp_snapshot.is_running:
    trigger_run_poll()
