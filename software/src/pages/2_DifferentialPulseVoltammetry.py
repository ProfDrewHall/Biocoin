import asyncio
import streamlit as st
import pandas as pd
import datetime
from io import BytesIO
import plotly.express as px

from gui_backend import require_backend_runtime, run_until_done_live

# -------------------------------------------------
# Streamlit Config
# -------------------------------------------------
st.set_page_config(page_title="BioCoin GUI", page_icon="🧫", layout="wide")
st.title("Differential Pulse Voltammetry (DPV)")
st.caption("Configure and run Differential Pulse Voltammetry experiments in real-time.")
require_backend_runtime()

from biocoin.device import BiocoinDevice
from biocoin.techniques.dpv import DifferentialPulseVoltammetry

# -------------------------------------------------
# Session-level async + device management
# -------------------------------------------------
if "loop" not in st.session_state:
    st.session_state.loop = asyncio.new_event_loop()
    asyncio.set_event_loop(st.session_state.loop)

if "device" not in st.session_state:
    st.session_state.device = BiocoinDevice()
    st.session_state.device_connected = False

if "dpv_running" not in st.session_state:
    st.session_state.dpv_running = False

if "dpv_experiments" not in st.session_state:
    st.session_state.dpv_experiments = {}


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
# Async DPV runner (robust)
# -------------------------------------------------
async def run_dpv_experiment(
    start_potential,
    end_potential,
    pulse_amplitude,
    step_potential,
    pulse_width,
    scan_rate,
    channel,
):
    device = await get_device()
    dpv = DifferentialPulseVoltammetry(device)

    await dpv.configure(
        E_start=start_potential,
        E_stop=end_potential,
        E_pulse=pulse_amplitude,
        E_step=step_potential,
        pulse_width=pulse_width,
        pulse_period=scan_rate,
        channel=channel,
        processing_interval=1.0,
        max_current=100.0,
    )

    placeholder_plot = st.empty()
    placeholder_progress = st.empty()

    df = pd.DataFrame(columns=["Potential (mV)", "Current (µA)"])
    progress_bar = placeholder_progress.progress(0.0, text="Running...")
    raw_samples = []
    render_count = 0

    def append_samples(samples):
        nonlocal df
        raw_samples.extend(samples)
        while len(raw_samples) // 2 > len(df):
            idx = len(df)
            if dpv.V is None or idx >= len(dpv.V):
                break
            current = raw_samples[2 * idx] - raw_samples[2 * idx + 1]
            df.loc[len(df)] = [dpv.V[idx], current]

    def draw_live_plot():
        nonlocal render_count
        if df.empty:
            return
        render_count += 1
        fig = px.line(
            df,
            x="Potential (mV)",
            y="Current (µA)",
            title="Live DPV Data",
        )
        placeholder_plot.plotly_chart(fig, use_container_width=True, key=f"dpv_live_plot_{render_count}")

    def on_samples(samples):
        append_samples(samples)
        draw_live_plot()

    def on_progress(elapsed, total):
        progress_bar.progress(
            min(elapsed / total, 1.0),
            text=f"Running... {elapsed:.1f} s / {total:.1f} s",
        )

    try:
        await run_until_done_live(
            dpv,
            device,
            dpv.duration,
            poll_interval=min(0.25, max(scan_rate / 2000.0, 0.05)),
            on_samples=on_samples,
            on_progress=on_progress,
        )
        if dpv.V is not None and len(df) != len(dpv.V):
            raise RuntimeError(f"Expected {len(dpv.V)} DPV points, received {len(df)}")
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
# Left – Settings
# -------------------------------------------------
with cols[0].container(border=True):
    st.subheader("Experiment Settings")

    start_potential = st.number_input("Start Potential (mV)", value=100.0, step=50.0)
    end_potential = st.number_input("End Potential (mV)", value=200.0, step=50.0)
    step_potential = st.number_input("Step Potential (mV)", value=50.0, step=10.0)
    pulse_amplitude = st.number_input("Pulse Amplitude (mV)", value=100.0, step=10.0)
    pulse_width = st.number_input("Pulse Width (ms)", value=150.0, step=10.0)
    scan_rate = st.number_input("Pulse Period (ms)", value=400.0, step=10.0)
    channel = st.selectbox("Channel", options=[0, 1, 2, 3])

    exp_name_input = st.text_input(
        "Experiment Name (optional)",
        value=f"DPV Experiment {len(st.session_state.dpv_experiments) + 1}",
    )

    run_button = st.button(
        "Run Experiment",
        use_container_width=True,
        disabled=st.session_state.dpv_running,
    )

    st.divider()

    if st.button("Reset Device Connection", use_container_width=True):
        run_async(reset_device())
        st.success("Device connection reset.")


# -------------------------------------------------
# Right – Results
# -------------------------------------------------
with cols[1].container(border=True):
    st.subheader("Results")

    # -----------------------
    # Run experiment
    # -----------------------
    if run_button and not st.session_state.dpv_running:
        st.session_state.dpv_running = True
        status = st.empty()

        try:
            status.info("Running DPV experiment...")
            df = run_async(
                run_dpv_experiment(
                    start_potential,
                    end_potential,
                    pulse_amplitude,
                    step_potential,
                    pulse_width,
                    scan_rate,
                    channel,
                )
            )

            exp_name = exp_name_input.strip() or f"DPV Experiment {len(st.session_state.dpv_experiments) + 1}"
            st.session_state.dpv_experiments[exp_name] = {
                "data": df,
                "meta": {
                    "Date": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "Start Potential (mV)": start_potential,
                    "End Potential (mV)": end_potential,
                    "Pulse Amplitude (mV)": pulse_amplitude,
                    "Pulse Width (ms)": pulse_width,
                    "Scan Rate (ms per step)": scan_rate,
                    "Channel": channel,
                    "Notes": "",
                },
            }

            status.success("Experiment completed successfully!")

        except Exception as e:
            status.error(f"Error: {e}")

        finally:
            st.session_state.dpv_running = False

    # -----------------------
    # Display experiments
    # -----------------------
    if st.session_state.dpv_experiments:
        exp_names = list(st.session_state.dpv_experiments.keys())
        selected_exps = st.multiselect("Select experiments to display", exp_names, default=exp_names)

        display_mode = st.radio("Display mode", ["Plot", "Table"], horizontal=True)

        if display_mode == "Plot":
            fig = px.line(title="DPV Results", labels={"x": "Potential (mV)", "y": "Current (µA)"})
            for name in selected_exps:
                df = st.session_state.dpv_experiments[name]["data"]
                fig.add_scatter(x=df["Potential (mV)"], y=df["Current (µA)"], name=name)
            st.plotly_chart(fig, use_container_width=True, key="dpv_results_plot")
        else:
            combined_df = pd.concat(
                [
                    data["data"].assign(Experiment=name)
                    for name, data in st.session_state.dpv_experiments.items()
                    if name in selected_exps
                ],
                ignore_index=True,
            )
            st.dataframe(combined_df, use_container_width=True)

        # -----------------------
        # Notes
        # -----------------------
        st.divider()
        note_exp = st.selectbox("Edit notes", exp_names)
        note = st.text_area(
            "Notes",
            st.session_state.dpv_experiments[note_exp]["meta"]["Notes"],
            height=150,
        )
        if st.button("Save Notes", use_container_width=True):
            st.session_state.dpv_experiments[note_exp]["meta"]["Notes"] = note
            st.success("Notes saved.")

        # -----------------------
        # Downloads
        # -----------------------
        st.divider()
        st.subheader("Download Data")

        selected_data = []
        for name in selected_exps:
            entry = st.session_state.dpv_experiments[name]
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
                    "DPV_selected_results.csv",
                    "text/csv",
                    use_container_width=True,
                )
            with col2:
                st.download_button(
                    "Download Excel",
                    excel_buffer.getvalue(),
                    "DPV_selected_results.xlsx",
                    "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
                    use_container_width=True,
                )
