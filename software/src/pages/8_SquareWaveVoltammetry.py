import asyncio
import streamlit as st
import pandas as pd
import datetime
from io import BytesIO
import plotly.express as px

from gui_backend import require_backend_runtime, run_until_done_live

# -------------------------------------------------
# Streamlit config
# -------------------------------------------------
st.set_page_config(page_title="Square Wave Voltammetry", page_icon="🧫", layout="wide")

st.title("Square Wave Voltammetry (SWV)")
st.caption("Configure and run Square Wave Voltammetry experiments in real time.")
require_backend_runtime()

from biocoin.device import BiocoinDevice
from biocoin.techniques.swv import SquareWaveVoltammetry

# -------------------------------------------------
# Session-level async + device management
# -------------------------------------------------
if "loop" not in st.session_state:
    st.session_state.loop = asyncio.new_event_loop()
    asyncio.set_event_loop(st.session_state.loop)

if "device" not in st.session_state:
    st.session_state.device = BiocoinDevice()
    st.session_state.device_connected = False

if "swv_running" not in st.session_state:
    st.session_state.swv_running = False

if "swv_experiments" not in st.session_state:
    st.session_state.swv_experiments = {}

# Helper to run async coroutines
def run_async(coro):
    return st.session_state.loop.run_until_complete(coro)

# Device management
async def get_device():
    if not st.session_state.device_connected:
        await st.session_state.device.connect()
        st.session_state.device_connected = True
    return st.session_state.device

async def reset_device():
    try:
        await st.session_state.device.disconnect()
    except Exception:
        pass
    st.session_state.device = BiocoinDevice()
    st.session_state.device_connected = False

# -------------------------------------------------
# Async SWV experiment runner
# -------------------------------------------------
async def run_swv(
    start_potential,
    end_potential,
    step_potential,
    pulse_amplitude,
    pulse_period,
    max_current,
    channel,
):
    device = await get_device()
    swv = SquareWaveVoltammetry(device)
    await swv.configure(
        processing_interval=1.0,
        max_current=max_current,
        E_start=start_potential,
        E_stop=end_potential,
        E_amplitude=pulse_amplitude,
        E_step=step_potential,
        pulse_period=pulse_period,
        channel=channel,
    )

    plot_placeholder = st.empty()
    progress_placeholder = st.empty()
    df = pd.DataFrame(columns=["Potential (mV)", "Differential Current (µA)"])
    progress_bar = progress_placeholder.progress(0.0, text="Running... 0%")
    raw_samples = []
    render_count = 0

    def append_samples(samples):
        nonlocal df
        raw_samples.extend(samples)
        while len(raw_samples) // 2 > len(df):
            idx = len(df)
            if swv.V is None or idx >= len(swv.V):
                break
            current = raw_samples[2 * idx + 1] - raw_samples[2 * idx]
            df.loc[len(df)] = [swv.V[idx], current]

    def draw_live_plot():
        nonlocal render_count
        if df.empty:
            return
        render_count += 1
        fig = px.line(df, x="Potential (mV)", y="Differential Current (µA)", title="Live SWV Data")
        plot_placeholder.plotly_chart(fig, use_container_width=True, key=f"swv_live_plot_{render_count}")

    def on_samples(samples):
        append_samples(samples)
        draw_live_plot()

    def on_progress(elapsed, total):
        progress_bar.progress(
            min(elapsed / total, 1.0),
            text=f"Running... {elapsed:.1f} s / {total:.1f} s",
        )

    def is_complete():
        return swv.V is not None and len(df) >= len(swv.V)

    try:
        await run_until_done_live(
            swv,
            device,
            swv.duration,
            poll_interval=min(0.25, max(pulse_period / 2000.0, 0.05)),
            on_samples=on_samples,
            on_progress=on_progress,
            is_complete=is_complete,
            max_polls=60,
        )
        if swv.V is not None and len(df) != len(swv.V):
            raise RuntimeError(f"Expected {len(swv.V)} SWV points, received {len(df)}")
        progress_bar.progress(1.0, text="Complete")
    finally:
        plot_placeholder.empty()
        progress_placeholder.empty()

    return df

# -------------------------------------------------
# Layout
# -------------------------------------------------
cols = st.columns([1, 2], gap="small")

# LEFT: Settings
with cols[0].container(border=True):
    st.subheader("Experiment Settings")

    start_potential = st.number_input("Start Potential (mV)", value=100.0, step=50.0)
    end_potential = st.number_input("End Potential (mV)", value=300.0, step=50.0)
    step_potential = st.number_input("Step Potential (mV)", min_value=1.0, value=25.0, step=5.0)
    pulse_amplitude = st.number_input("Square Wave Amplitude (mV)", min_value=1.0, value=50.0, step=5.0)
    pulse_period = st.number_input("Square Wave Period (ms)", min_value=10.0, value=200.0, step=10.0)
    max_current = st.number_input("Max Current (µA)", min_value=1.0, max_value=3000.0, value=100.0, step=50.0)
    channel = st.selectbox("Channel", options=[0, 1, 2, 3])

    exp_name_input = st.text_input(
        "Experiment Name (optional)",
        value=f"SWV Experiment {len(st.session_state.swv_experiments) + 1}",
    )

    run_button = st.button(
        "Run Experiment",
        use_container_width=True,
        disabled=st.session_state.swv_running,
    )

    st.divider()
    if st.button("Reset Device Connection", use_container_width=True):
        run_async(reset_device())
        st.success("Device connection reset.")

# RIGHT: Results
with cols[1].container(border=True):
    st.subheader("Results")

    if run_button and not st.session_state.swv_running:
        st.session_state.swv_running = True
        status = st.empty()

        try:
            status.info("Running SWV experiment...")
            df = run_async(
                run_swv(
                    start_potential,
                    end_potential,
                    step_potential,
                    pulse_amplitude,
                    pulse_period,
                    max_current,
                    channel,
                )
            )

            exp_name = exp_name_input.strip() or f"SWV Experiment {len(st.session_state.swv_experiments) + 1}"
            st.session_state.swv_experiments[exp_name] = {
                "data": df,
                "meta": {
                    "Date": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "Start Potential (mV)": start_potential,
                    "End Potential (mV)": end_potential,
                    "Step Potential (mV)": step_potential,
                    "Square Wave Amplitude (mV)": pulse_amplitude,
                    "Square Wave Period (ms)": pulse_period,
                    "Max Current (µA)": max_current,
                    "Channel": channel,
                    "Notes": "",
                },
            }
            status.success("Experiment completed successfully!")

        except Exception as e:
            status.error(f"Error: {e}")

        finally:
            st.session_state.swv_running = False

    # Display experiments
    if st.session_state.swv_experiments:
        exp_names = list(st.session_state.swv_experiments.keys())
        selected_exps = st.multiselect("Select experiments", exp_names, default=exp_names)

        # Plot / Table toggle
        display_mode = st.radio("Display mode", ["Plot", "Table"], horizontal=True)
        if display_mode == "Plot":
            fig = px.line(title="SWV Results")
            for name in selected_exps:
                d = st.session_state.swv_experiments[name]["data"]
                fig.add_scatter(x=d["Potential (mV)"], y=d["Differential Current (µA)"], mode="lines", name=name)
            st.plotly_chart(fig, use_container_width=True, key="swv_results_plot")
        else:
            combined_df = pd.concat(
                [v["data"].assign(Experiment=name) for name, v in st.session_state.swv_experiments.items() if name in selected_exps],
                ignore_index=True,
            )
            st.dataframe(combined_df, use_container_width=True)

        # Notes
        st.divider()
        note_exp = st.selectbox("Edit notes", exp_names)
        note = st.text_area("Notes", st.session_state.swv_experiments[note_exp]["meta"]["Notes"], height=150)
        if st.button("Save Notes", use_container_width=True):
            st.session_state.swv_experiments[note_exp]["meta"]["Notes"] = note
            st.success("Notes saved.")

        # Downloads
        st.divider()
        st.subheader("Download Data")
        selected_data = []
        for name in selected_exps:
            entry = st.session_state.swv_experiments[name]
            meta_df = pd.DataFrame(entry["meta"].items(), columns=["Parameter", "Value"])
            selected_data.append((name, meta_df, entry["data"], entry["meta"]))

        if selected_data:
            # CSV
            csv_parts = []
            for name, _, df, meta in selected_data:
                header = f"# Experiment: {name}\n" + "\n".join([f"# {k}: {v}" for k, v in meta.items()])
                csv_parts.append(f"{header}\n\n{df.to_csv(index=False)}")
            csv_output = "\n\n".join(csv_parts)

            # Excel
            excel_buffer = BytesIO()
            with pd.ExcelWriter(excel_buffer, engine="xlsxwriter") as writer:
                for name, meta_df, df, _ in selected_data:
                    sheet = name[:31]
                    meta_df.to_excel(writer, sheet_name=sheet, index=False)
                    df.to_excel(writer, sheet_name=sheet, startrow=len(meta_df) + 2, index=False)
                    ws = writer.sheets[sheet]
                    for i in range(len(meta_df.columns) + len(df.columns)):
                        ws.set_column(i, i, 20)

            col1, col2 = st.columns(2)
            with col1:
                st.download_button(
                    "Download CSV",
                    csv_output.encode("utf-8"),
                    "SWV_results.csv",
                    "text/csv",
                    use_container_width=True,
                )
            with col2:
                st.download_button(
                    "Download Excel",
                    excel_buffer.getvalue(),
                    "SWV_results.xlsx",
                    "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
                    use_container_width=True,
                )
