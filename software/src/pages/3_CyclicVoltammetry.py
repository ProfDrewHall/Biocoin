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
st.set_page_config(page_title="Cyclic Voltammetry", page_icon="🔁", layout="wide")
st.title("Cyclic Voltammetry (CV)")
st.caption("Configure and run cyclic voltammetry experiments in real time.")
require_backend_runtime()

from biocoin.device import BiocoinDevice
from biocoin.techniques.cv import CyclicVoltammetry

# -------------------------------------------------
# Session-level async + device management
# -------------------------------------------------
if "loop" not in st.session_state:
    st.session_state.loop = asyncio.new_event_loop()
    asyncio.set_event_loop(st.session_state.loop)

if "device" not in st.session_state:
    st.session_state.device = BiocoinDevice()
    st.session_state.device_connected = False

if "cv_running" not in st.session_state:
    st.session_state.cv_running = False

if "cv_experiments" not in st.session_state:
    st.session_state.cv_experiments = {}


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
# Async CV runner (robust)
# -------------------------------------------------
async def run_cv_experiment(
    processing_interval,
    max_current,
    E_start,
    E_vertex1,
    E_vertex2,
    E_step,
    pulse_width,
    channel,
):
    device = await get_device()
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

    placeholder_plot = st.empty()
    placeholder_progress = st.empty()
    df = pd.DataFrame(columns=["Voltage (mV)", "Current (µA)"])
    progress_bar = placeholder_progress.progress(0.0, text="Running...")
    render_count = 0

    def append_samples(samples):
        nonlocal df
        for current in samples:
            idx = len(df)
            if cv.V is None or idx >= len(cv.V):
                break
            df.loc[len(df)] = [cv.V[idx], current]

    def draw_live_plot():
        nonlocal render_count
        if df.empty:
            return
        render_count += 1
        fig = px.line(
            df,
            x="Voltage (mV)",
            y="Current (µA)",
            title="Live Cyclic Voltammetry",
        )
        placeholder_plot.plotly_chart(fig, use_container_width=True, key=f"cv_live_plot_{render_count}")

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
            cv,
            device,
            cv.duration,
            poll_interval=min(0.25, max(pulse_width / 2000.0, 0.05)),
            on_samples=on_samples,
            on_progress=on_progress,
        )
        if cv.V is not None and len(df) != len(cv.V):
            raise RuntimeError(f"Expected {len(cv.V)} CV points, received {len(df)}")
        progress_bar.progress(1.0, text="Complete")
    finally:
        placeholder_plot.empty()
        placeholder_progress.empty()

    return df, cv.duration


# -------------------------------------------------
# Layout
# -------------------------------------------------
cols = st.columns([1, 2], gap="small")

# -------------------------------------------------
# Left – Settings
# -------------------------------------------------
with cols[0].container(border=True):
    st.subheader("Experiment Settings")

    processing_interval = st.number_input(
        "Processing Interval (s)", min_value=0.01, value=0.1, step=0.01
    )
    max_current = st.number_input(
        "Max Current (µA)", min_value=1.0, max_value=3000.0, value=100.0
    )
    E_start = st.number_input("E Start (mV)", value=0.0)
    E_vertex1 = st.number_input("Vertex 1 (mV)", value=-500.0)
    E_vertex2 = st.number_input("Vertex 2 (mV)", value=500.0)
    E_step = st.number_input("Step Size (mV)", min_value=0.6, value=5.0)
    pulse_width = st.number_input("Pulse Width (ms)", min_value=3.0, value=50.0)
    channel = st.selectbox("Channel", options=[0, 1, 2, 3])

    exp_name_input = st.text_input(
        "Experiment Name (optional)",
        value=f"CV Experiment {len(st.session_state.cv_experiments) + 1}",
    )

    run_button = st.button(
        "Run Experiment",
        use_container_width=True,
        disabled=st.session_state.cv_running,
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
    if run_button and not st.session_state.cv_running:
        st.session_state.cv_running = True
        status = st.empty()

        try:
            status.info("Running CV experiment...")
            df, duration = run_async(
                run_cv_experiment(
                    processing_interval,
                    max_current,
                    E_start,
                    E_vertex1,
                    E_vertex2,
                    E_step,
                    pulse_width,
                    channel,
                )
            )

            exp_name = exp_name_input.strip() or f"CV Experiment {len(st.session_state.cv_experiments) + 1}"
            st.session_state.cv_experiments[exp_name] = {
                "data": df,
                "meta": {
                    "Date": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "Processing Interval (s)": processing_interval,
                    "Max Current (µA)": max_current,
                    "E Start (mV)": E_start,
                    "E Vertex 1 (mV)": E_vertex1,
                    "E Vertex 2 (mV)": E_vertex2,
                    "E Step (mV)": E_step,
                    "Pulse Width (ms)": pulse_width,
                    "Channel": channel,
                    "Duration (s)": round(duration, 3),
                    "Notes": "",
                },
            }

            status.success("Experiment completed successfully!")

        except Exception as e:
            status.error(f"Error: {e}")

        finally:
            st.session_state.cv_running = False

    # -----------------------
    # Display experiments
    # -----------------------
    if st.session_state.cv_experiments:
        exp_names = list(st.session_state.cv_experiments.keys())
        selected_exps = st.multiselect("Select experiments", exp_names, default=exp_names)

        display_mode = st.radio("Display mode", ["Plot", "Table"], horizontal=True)

        if display_mode == "Plot":
            fig = px.line(title="Cyclic Voltammetry Results")
            for name in selected_exps:
                df = st.session_state.cv_experiments[name]["data"]
                fig.add_scatter(x=df["Voltage (mV)"], y=df["Current (µA)"], name=name)
            st.plotly_chart(fig, use_container_width=True, key="cv_results_plot")
        else:
            combined_df = pd.concat(
                [
                    data["data"].assign(Experiment=name)
                    for name, data in st.session_state.cv_experiments.items()
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
            st.session_state.cv_experiments[note_exp]["meta"]["Notes"],
            height=150,
        )
        if st.button("Save Notes", use_container_width=True):
            st.session_state.cv_experiments[note_exp]["meta"]["Notes"] = note
            st.success("Notes saved.")

        # -----------------------
        # Downloads
        # -----------------------
        st.divider()
        st.subheader("Download Data")

        selected_data = []
        for name in selected_exps:
            entry = st.session_state.cv_experiments[name]
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
                    "CV_results.csv",
                    "text/csv",
                    use_container_width=True,
                )
            with col2:
                st.download_button(
                    "Download Excel",
                    excel_buffer.getvalue(),
                    "CV_results.xlsx",
                    "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
                    use_container_width=True,
                )
