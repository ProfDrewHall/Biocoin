import asyncio
import datetime

import pandas as pd
import plotly.express as px
import streamlit as st

from biocoin.techniques.swv import SquareWaveVoltammetry
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
    run_until_done_live,
    start_background_run,
    trigger_run_poll,
)

PAGE = PAGE_METADATA['swv']
CURRENT_LABEL = 'Differential Current (µA)'

st.set_page_config(page_title=PAGE.page_title, page_icon=PAGE.icon, layout='wide')
st.title(PAGE.title)
st.caption(PAGE.caption)
require_backend_runtime()
require_device_connection()

initialize_experiment_state('swv')


async def run_swv_experiment(
    controller: BackgroundTechniqueRun,
    device,
    *,
    start_potential: float,
    end_potential: float,
    step_potential: float,
    pulse_amplitude: float,
    pulse_period: float,
    processing_interval: float,
    max_current: float,
    channel: int,
) -> pd.DataFrame:
    swv = SquareWaveVoltammetry(device)
    await swv.configure(
        processing_interval=processing_interval,
        max_current=max_current,
        E_start=start_potential,
        E_stop=end_potential,
        E_amplitude=pulse_amplitude,
        E_step=step_potential,
        pulse_period=pulse_period,
        channel=channel,
    )

    controller.bind_abort(asyncio.get_running_loop(), lambda: device.write_ctrl_command(swv.Command.STOP))

    df = pd.DataFrame(columns=['Potential (mV)', CURRENT_LABEL])
    raw_samples: list[float] = []

    def append_samples(samples) -> None:
        nonlocal df
        raw_samples.extend(samples)
        while len(raw_samples) // 2 > len(df):
            idx = len(df)
            if swv.V is None or idx >= len(swv.V):
                break
            current = raw_samples[2 * idx + 1] - raw_samples[2 * idx]
            df.loc[len(df)] = [swv.V[idx], current]
        if not df.empty:
            controller.set_live_data(df.copy())

    def on_progress(elapsed: float, total: float) -> None:
        controller.set_progress(min(elapsed / total, 1.0), f'Running... {elapsed:.1f} s / {total:.1f} s')

    def is_complete() -> bool:
        return swv.V is not None and len(df) >= len(swv.V)

    await run_until_done_live(
        swv,
        device,
        swv.duration,
        poll_interval=min(0.25, max(pulse_period / 2000.0, 0.05)),
        on_samples=append_samples,
        on_progress=on_progress,
        is_complete=is_complete,
        max_polls=60,
        should_stop=controller.should_stop,
    )
    controller.set_live_data(df.copy())

    if controller.should_stop():
        raise TechniqueAbortRequested
    if swv.V is not None and len(df) != len(swv.V):
        raise RuntimeError(f'Expected {len(swv.V)} SWV points, received {len(df)}')

    return df


def finalize_swv_run() -> BackgroundTechniqueRun | None:
    return finalize_experiment_run(
        'swv',
        aborted_message='Square Wave Voltammetry experiment aborted.',
    )


def start_swv_run(
    device,
    *,
    start_potential: float,
    end_potential: float,
    step_potential: float,
    pulse_amplitude: float,
    pulse_period: float,
    processing_interval: float,
    max_current: float,
    channel: int,
    exp_name: str,
) -> None:
    st.session_state.swv_pending_experiment = {
        'name': exp_name,
        'meta': {
            'Date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'Start Potential (mV)': start_potential,
            'End Potential (mV)': end_potential,
            'Step Potential (mV)': step_potential,
            'Square Wave Amplitude (mV)': pulse_amplitude,
            'Square Wave Period (ms)': pulse_period,
            'Processing Interval (s)': processing_interval,
            'Max Current (µA)': max_current,
            'Channel': channel,
            'Notes': '',
        },
    }
    st.session_state.swv_message = None
    st.session_state.swv_run = start_background_run(
        lambda controller: run_swv_experiment(
            controller,
            device,
            start_potential=start_potential,
            end_potential=end_potential,
            step_potential=step_potential,
            pulse_amplitude=pulse_amplitude,
            pulse_period=pulse_period,
            processing_interval=processing_interval,
            max_current=max_current,
            channel=channel,
        )
    )


swv_run = finalize_swv_run()
swv_snapshot = swv_run.snapshot() if swv_run is not None else None

cols = st.columns([1, 2], gap='small')

with cols[0].container(border=True):
    st.subheader('Experiment Settings')
    start_potential = st.number_input('Start Potential (mV)', value=100.0, step=50.0)
    end_potential = st.number_input('End Potential (mV)', value=300.0, step=50.0)
    step_potential = st.number_input('Step Potential (mV)', min_value=1.0, value=25.0, step=5.0)
    pulse_amplitude = st.number_input('Square Wave Amplitude (mV)', min_value=1.0, value=50.0, step=5.0)
    pulse_period = st.number_input('Square Wave Period (ms)', min_value=10.0, value=200.0, step=10.0)
    max_current = st.number_input('Max Current (µA)', min_value=1.0, max_value=3000.0, value=100.0, step=50.0)
    channel = st.selectbox('Channel', options=[0, 1, 2, 3])

    processing_interval_min = max(pulse_period / 2000.0, 0.01)
    with st.expander('Advanced Settings'):
        processing_interval = st.number_input(
            'Processing Interval (s)',
            min_value=processing_interval_min,
            value=max(0.1, processing_interval_min),
            step=0.01,
        )

    exp_name_input = st.text_input(
        'Experiment Name (optional)',
        value=f'SWV Experiment {len(st.session_state.swv_experiments) + 1}',
        disabled=swv_snapshot is not None and swv_snapshot.is_running,
    )

    action_label = 'Abort Experiment' if swv_snapshot is not None and swv_snapshot.is_running else 'Run Experiment'
    action_clicked = st.button(action_label, width='stretch', disabled=not st.session_state.connected)

    if action_clicked:
        if swv_snapshot is not None and swv_snapshot.is_running:
            swv_run.request_abort()
        else:
            exp_name = exp_name_input.strip() or f'SWV Experiment {len(st.session_state.swv_experiments) + 1}'
            start_swv_run(
                st.session_state.biocoin_device,
                start_potential=start_potential,
                end_potential=end_potential,
                step_potential=step_potential,
                pulse_amplitude=pulse_amplitude,
                pulse_period=pulse_period,
                processing_interval=processing_interval,
                max_current=max_current,
                channel=channel,
                exp_name=exp_name,
            )
        st.rerun()


with cols[1].container(border=True):
    st.subheader('Results')

    if st.session_state.swv_message is not None:
        level, message = st.session_state.swv_message
        getattr(st, level)(message)

    if swv_snapshot is not None and swv_snapshot.is_running:
        if swv_snapshot.abort_requested:
            st.warning(swv_snapshot.status_text)
        else:
            st.info(swv_snapshot.status_text)
        st.progress(swv_snapshot.progress, text=swv_snapshot.status_text)
        live_df = swv_snapshot.live_data
        if isinstance(live_df, pd.DataFrame) and not live_df.empty:
            fig = px.line(live_df, x='Potential (mV)', y=CURRENT_LABEL, title='Live SWV Data')
            st.plotly_chart(fig, width='stretch', key='swv_live_plot')

    if st.session_state.swv_experiments:
        exp_names = list(st.session_state.swv_experiments.keys())
        selected_exps = st.multiselect('Select experiments', exp_names, default=exp_names)
        display_mode = st.radio('Display mode', ['Plot', 'Table'], horizontal=True)

        if display_mode == 'Plot':
            fig = px.line(title='SWV Results')
            for name in selected_exps:
                df = st.session_state.swv_experiments[name].data
                fig.add_scatter(x=df['Potential (mV)'], y=df[CURRENT_LABEL], mode='lines', name=name)
            st.plotly_chart(fig, width='stretch', key='swv_results_plot')
        else:
            combined_df = pd.concat(
                [
                    record.data.assign(Experiment=name)
                    for name, record in st.session_state.swv_experiments.items()
                    if name in selected_exps
                ],
                ignore_index=True,
            )
            st.dataframe(combined_df, width='stretch')

        st.divider()
        render_notes_editor(st.session_state.swv_experiments, key_prefix='swv')

        st.divider()
        st.subheader('Download Data')
        render_experiment_downloads(
            st.session_state.swv_experiments,
            selected_exps,
            csv_filename='SWV_results.csv',
            excel_filename='SWV_results.xlsx',
            key_prefix='swv',
        )


if swv_snapshot is not None and swv_snapshot.is_running:
    trigger_run_poll()

