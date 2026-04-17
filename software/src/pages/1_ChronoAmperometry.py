import asyncio
import streamlit as st
import pandas as pd
import datetime
from io import BytesIO
import plotly.express as px

from gui_backend import require_backend_runtime

# -------------------------------------------------
# Streamlit Config
# -------------------------------------------------
st.set_page_config(page_title="ChronoAmperometry", page_icon="⚡", layout="wide")
st.title("ChronoAmperometry (CA)")
st.caption("Configure and run ChronoAmperometry experiments in real time.")
require_backend_runtime()

from biocoin.device import BiocoinDevice
from biocoin.techniques.ca import ChronoAmperometry

# -------------------------------------------------
# Session-level async + device management
# -------------------------------------------------
if "loop" not in st.session_state:
    st.session_state.loop = asyncio.new_event_loop()
    asyncio.set_event_loop(st.session_state.loop)

if "device" not in st.session_state:
    st.session_state.device = BiocoinDevice()
    st.session_state.device_connected = False

if "ca_running" not in st.session_state:
    st.session_state.ca_running = False

if "ca_experiments" not in st.session_state:
    st.session_state.ca_experiments = {}


def run_async(coro):
    return st.session_state.loop.run_until_complete(coro)


async def get_device():
    device = st.session_state.device
    if not st.session_state.device_connected:
        await device.connect()
        st.session_state.device_connected = True
    return device


async def reset_device():
    try:
        await st.session_state.device.disconnect()
    except Exception:
        pass
    st.session_state.device = BiocoinDevice()
    st.session_state.device_connected = False


# -------------------------------------------------
# Async CA runner (safe + reusable)
# -------------------------------------------------
async def run_ca_experiment(
    sampling_interval,
    processing_interval,
    max_current,
    pulse_potential,
    channel,
    duration,
):
    device = await get_device()
    ca = ChronoAmperometry(device)

    await ca.configure(
        sampling_interval=sampling_interval,
        processing_interval=processing_interval,
        max_current=max_current,
        pulse_potential=pulse_potential,
        channel=channel,
    )

    placeholder_plot = st.empty()
    placeholder_progress = st.empty()

    df = pd.DataFrame(columns=["Time (s)", "Current (µA)"])
    progress_bar = placeholder_progress.progress(0.0, text="Running...")
    sample_count = 0
    render_count = 0
    poll_interval = min(0.25, max(sampling_interval / 2.0, 0.05))

    def append_samples(samples):
        nonlocal df, sample_count
        for current in samples:
            df.loc[len(df)] = [sample_count * sampling_interval, current]
            sample_count += 1

    def draw_live_plot():
        nonlocal render_count
        if df.empty:
            return
        render_count += 1
        fig = px.line(
            df,
            x="Time (s)",
            y="Current (µA)",
            title="Live ChronoAmperometry Data",
        )
        placeholder_plot.plotly_chart(fig, use_container_width=True, key=f"ca_live_plot_{render_count}")

    try:
        await ca.start()
        loop = asyncio.get_running_loop()
        start_time = loop.time()
        device_started = False

        try:
            await device.write_ctrl_command(ca.Command.START)
            device_started = True

            while True:
                elapsed = loop.time() - start_time
                if elapsed >= duration:
                    break

                await asyncio.sleep(min(poll_interval, duration - elapsed))
                append_samples(ca._drain_queue())
                draw_live_plot()

                elapsed = min(loop.time() - start_time, duration)
                progress_bar.progress(
                    min(elapsed / duration, 1.0),
                    text=f"Running... {elapsed:.1f} s / {duration} s",
                )

        finally:
            if device_started:
                try:
                    await device.write_ctrl_command(ca.Command.STOP)
                    await asyncio.sleep(1.0)
                    append_samples(ca._drain_queue())
                    draw_live_plot()
                except Exception:
                    pass
            await ca.stop()

        progress_bar.progress(1.0, text="Complete")
    finally:
        placeholder_plot.empty()
        placeholder_progress.empty()

    return df


# -------------------------------------------------
# Layout
# -------------------------------------------------
cols = st.columns([1, 2], gap="small")

# -------------------------------------------------
# Left Column – Settings
# -------------------------------------------------
with cols[0].container(border=True):
    st.subheader("Experiment Settings")

    sampling_interval = st.number_input("Sampling Interval (s)", min_value=0.1, value=1.0)
    processing_interval = st.number_input("Processing Interval (s)", min_value=0.1, value=1.0)
    max_current = st.number_input("Max Current (µA)", min_value=1.0, max_value=10000.0, value=100.0)
    pulse_potential = st.number_input("Pulse Potential (mV)", min_value=-1000.0, max_value=1000.0, value=200.0)
    channel = st.selectbox("Channel", options=[0, 1, 2, 3])
    duration = st.number_input("Duration (s)", min_value=1, max_value=600, value=15)

    exp_name_input = st.text_input(
        "Experiment Name (optional)",
        value=f"Experiment {len(st.session_state.ca_experiments) + 1}",
    )

    run_button = st.button(
        "Run Experiment",
        use_container_width=True,
        disabled=st.session_state.ca_running,
    )

    st.divider()

    if st.button("Reset Device Connection", use_container_width=True):
        run_async(reset_device())
        st.success("Device connection reset.")


# -------------------------------------------------
# Right Column – Results
# -------------------------------------------------
with cols[1].container(border=True):
    st.subheader("Results")

    # -----------------------
    # Run experiment
    # -----------------------
    if run_button and not st.session_state.ca_running:
        st.session_state.ca_running = True
        status = st.empty()

        try:
            status.info("Running CA experiment...")
            df = run_async(
                run_ca_experiment(
                    sampling_interval,
                    processing_interval,
                    max_current,
                    pulse_potential,
                    channel,
                    duration,
                )
            )

            exp_name = exp_name_input.strip() or f"Experiment {len(st.session_state.ca_experiments) + 1}"
            st.session_state.ca_experiments[exp_name] = {
                "data": df,
                "meta": {
                    "Date": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "Sampling Interval (s)": sampling_interval,
                    "Processing Interval (s)": processing_interval,
                    "Max Current (µA)": max_current,
                    "Pulse Potential (mV)": pulse_potential,
                    "Channel": channel,
                    "Duration (s)": duration,
                    "Notes": "",
                },
            }

            status.success("Experiment completed successfully!")

        except Exception as e:
            status.error(f"Error: {e}")

        finally:
            st.session_state.ca_running = False

    # -----------------------
    # Display experiments
    # -----------------------
    if st.session_state.ca_experiments:
        exp_names = list(st.session_state.ca_experiments.keys())
        selected_exps = st.multiselect("Select experiments", exp_names, default=exp_names)

        fig = px.line(title="ChronoAmperometry Results")
        for name in selected_exps:
            df = st.session_state.ca_experiments[name]["data"]
            fig.add_scatter(x=df["Time (s)"], y=df["Current (µA)"], name=name)

        st.plotly_chart(fig, use_container_width=True, key="ca_results_plot")

        # Notes
        st.divider()
        note_exp = st.selectbox("Edit notes", exp_names)
        note = st.text_area(
            "Notes",
            st.session_state.ca_experiments[note_exp]["meta"]["Notes"],
            height=120,
        )
        if st.button("Save Notes", use_container_width=True):
            st.session_state.ca_experiments[note_exp]["meta"]["Notes"] = note
            st.success("Notes saved.")
