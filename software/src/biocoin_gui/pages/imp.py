import asyncio
import datetime

import pandas as pd
import plotly.graph_objects as go
import streamlit as st

from biocoin.techniques.impedance import Impedance
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

PAGE = PAGE_METADATA['imp']
MAGNITUDE_LABEL = 'Magnitude (Ω)'
PHASE_LABEL = 'Phase (deg)'

st.set_page_config(page_title=PAGE.page_title, page_icon=PAGE.icon, layout='wide')
st.title(PAGE.title)
st.caption(PAGE.caption)
require_backend_runtime()
require_device_connection()

initialize_experiment_state('imp')


def build_imp_figure(df: pd.DataFrame, title: str) -> go.Figure:
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=df['Index'], y=df[MAGNITUDE_LABEL], name=MAGNITUDE_LABEL, yaxis='y1'))
    fig.add_trace(go.Scatter(x=df['Index'], y=df[PHASE_LABEL], name=PHASE_LABEL, yaxis='y2'))
    fig.update_layout(
        title=title,
        xaxis_title='Sample Index',
        yaxis={'title': MAGNITUDE_LABEL},
        yaxis2={'title': PHASE_LABEL, 'overlaying': 'y', 'side': 'right'},
    )
    return fig


async def run_imp_experiment(
    controller: BackgroundTechniqueRun,
    device,
    *,
    sampling_interval: float,
    processing_interval: float,
    imp_4wire: bool,
    ac_coupled: bool,
    max_current: float,
    E_ac: float,
    frequency: float,
    duration: int,
) -> pd.DataFrame:
    imp = Impedance(device)
    await imp.configure(
        sampling_interval=sampling_interval,
        processing_interval=processing_interval,
        IMP_4wire=imp_4wire,
        AC_coupled=ac_coupled,
        max_current=max_current,
        E_ac=E_ac,
        frequency=frequency,
    )

    controller.bind_abort(asyncio.get_running_loop(), lambda: device.write_ctrl_command(imp.Command.STOP))

    df = pd.DataFrame(columns=['Index', MAGNITUDE_LABEL, PHASE_LABEL])

    def append_samples(samples) -> None:
        nonlocal df
        for magnitude, phase in samples:
            df.loc[len(df)] = [len(df), magnitude, phase]
        if not df.empty:
            controller.set_live_data(df.copy())

    def on_progress(elapsed: float, total: float) -> None:
        controller.set_progress(min(elapsed / total, 1.0), f'Running... {elapsed:.1f} s / {total} s')

    await run_fixed_duration_live(
        imp,
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
    if df.empty:
        raise RuntimeError('No impedance data was received from the device.')

    return df


def finalize_imp_run() -> BackgroundTechniqueRun | None:
    return finalize_experiment_run(
        'imp',
        aborted_message='Impedance experiment aborted.',
    )


def start_imp_run(
    device,
    *,
    sampling_interval: float,
    processing_interval: float,
    imp_4wire: bool,
    ac_coupled: bool,
    max_current: float,
    E_ac: float,
    frequency: float,
    duration: int,
    exp_name: str,
) -> None:
    st.session_state.imp_pending_experiment = {
        'name': exp_name,
        'meta': {
            'Date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'Sampling Interval (s)': sampling_interval,
            'Processing Interval (s)': processing_interval,
            '4-Wire': imp_4wire,
            'AC Coupled': ac_coupled,
            'Max Current (µA)': max_current,
            'AC Amplitude (mV)': E_ac,
            'Frequency (Hz)': frequency,
            'Duration (s)': duration,
            'Notes': '',
        },
    }
    st.session_state.imp_message = None
    st.session_state.imp_run = start_background_run(
        lambda controller: run_imp_experiment(
            controller,
            device,
            sampling_interval=sampling_interval,
            processing_interval=processing_interval,
            imp_4wire=imp_4wire,
            ac_coupled=ac_coupled,
            max_current=max_current,
            E_ac=E_ac,
            frequency=frequency,
            duration=duration,
        )
    )


imp_run = finalize_imp_run()
imp_snapshot = imp_run.snapshot() if imp_run is not None else None

cols = st.columns([1, 2], gap='small')

with cols[0].container(border=True):
    st.subheader('Experiment Settings')
    imp_4wire = st.checkbox('4-Wire Measurement', value=True)
    ac_coupled = st.checkbox('AC Coupled', value=True)
    max_current = st.number_input('Max Current (µA)', 1.0, 3000.0, value=1000.0)
    E_ac = st.number_input('AC Amplitude (mV)', 1.0, 2200.0, value=100.0)
    frequency = st.number_input('Frequency (Hz)', 0.1, value=1000.0)
    duration = st.number_input('Duration (s)', 1, 300, value=10)

    with st.expander('Advanced Settings'):
        sampling_interval = st.number_input('Sampling Interval (s)', 0.01, value=0.1)
        processing_interval = st.number_input('Processing Interval (s)', 0.01, value=0.1)

    exp_name_input = st.text_input(
        'Experiment Name (optional)',
        value=f'IMP Experiment {len(st.session_state.imp_experiments) + 1}',
        disabled=imp_snapshot is not None and imp_snapshot.is_running,
    )

    action_label = 'Abort Experiment' if imp_snapshot is not None and imp_snapshot.is_running else 'Run Experiment'
    action_clicked = st.button(action_label, width='stretch', disabled=not st.session_state.connected)

    if action_clicked:
        if imp_snapshot is not None and imp_snapshot.is_running:
            imp_run.request_abort()
        else:
            exp_name = exp_name_input.strip() or f'IMP Experiment {len(st.session_state.imp_experiments) + 1}'
            start_imp_run(
                st.session_state.biocoin_device,
                sampling_interval=sampling_interval,
                processing_interval=processing_interval,
                imp_4wire=imp_4wire,
                ac_coupled=ac_coupled,
                max_current=max_current,
                E_ac=E_ac,
                frequency=frequency,
                duration=duration,
                exp_name=exp_name,
            )
        st.rerun()


with cols[1].container(border=True):
    st.subheader('Results')

    if st.session_state.imp_message is not None:
        level, message = st.session_state.imp_message
        getattr(st, level)(message)

    if imp_snapshot is not None and imp_snapshot.is_running:
        if imp_snapshot.abort_requested:
            st.warning(imp_snapshot.status_text)
        else:
            st.info(imp_snapshot.status_text)
        st.progress(imp_snapshot.progress, text=imp_snapshot.status_text)
        live_df = imp_snapshot.live_data
        if isinstance(live_df, pd.DataFrame) and not live_df.empty:
            st.plotly_chart(
                build_imp_figure(live_df, 'Live Impedance Measurement'),
                width='stretch',
                key='imp_live_plot',
            )

    if st.session_state.imp_experiments:
        exp_names = list(st.session_state.imp_experiments.keys())
        selected_exps = st.multiselect('Select experiments', exp_names, default=exp_names)
        display_mode = st.radio('Display mode', ['Plot', 'Table'], horizontal=True)

        if display_mode == 'Plot':
            fig = go.Figure()
            for name in selected_exps:
                df = st.session_state.imp_experiments[name].data
                fig.add_trace(go.Scatter(x=df['Index'], y=df[MAGNITUDE_LABEL], name=f'{name} | Mag', yaxis='y1'))
                fig.add_trace(go.Scatter(x=df['Index'], y=df[PHASE_LABEL], name=f'{name} | Phase', yaxis='y2'))
            fig.update_layout(
                title='Impedance Results',
                xaxis_title='Sample Index',
                yaxis={'title': MAGNITUDE_LABEL},
                yaxis2={'title': PHASE_LABEL, 'overlaying': 'y', 'side': 'right'},
            )
            st.plotly_chart(fig, width='stretch', key='imp_results_plot')
        else:
            combined_df = pd.concat(
                [
                    record.data.assign(Experiment=name)
                    for name, record in st.session_state.imp_experiments.items()
                    if name in selected_exps
                ],
                ignore_index=True,
            )
            st.dataframe(combined_df, width='stretch')

        st.divider()
        render_notes_editor(st.session_state.imp_experiments, key_prefix='imp')

        st.divider()
        st.subheader('Download Data')
        render_experiment_downloads(
            st.session_state.imp_experiments,
            selected_exps,
            csv_filename='IMP_results.csv',
            excel_filename='IMP_results.xlsx',
            key_prefix='imp',
        )


if imp_snapshot is not None and imp_snapshot.is_running:
    trigger_run_poll()

