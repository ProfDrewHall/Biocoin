import asyncio
import streamlit as st
import pandas as pd
import datetime
from io import BytesIO
import plotly.express as px

from gui_backend import require_backend_runtime

# -----------------------------
# Streamlit config
# -----------------------------
st.set_page_config(
    page_title="Open-Circuit Potential",
    page_icon="🔋",
    layout="wide",
)
st.title("Open-Circuit Potential (OCP)")
st.caption("Measure open-circuit potential as a function of time.")
require_backend_runtime()

from biocoin.device import BiocoinDevice
from biocoin.techniques.ocp import OpenCircuitPotential

# -----------------------------
# Session-level async + device
# -----------------------------
if "loop" not in st.session_state:
    st.session_state.loop = asyncio.new_event_loop()
    asyncio.set_event_loop(st.session_state.loop)

if "device" not in st.session_state:
    st.session_state.device = BiocoinDevice()
    st.session_state.device_connected = False

if "ocp_running" not in st.session_state:
    st.session_state.ocp_running = False

if "ocp_experiments" not in st.session_state:
    st.session_state.ocp_experiments = {}


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


# -----------------------------
# Async OCP runner
# -----------------------------
async def run_ocp_experiment(
    sampling_interval,
    processing_interval,
    channel,
    duration,
):
    device = await get_device()
    ocp = OpenCircuitPotential(device)
    await ocp.configure(
        sampling_interval=sampling_interval,
        processing_interval=processing_interval,
        channel=channel,
    )

    placeholder_plot = st.empty()
    placeholder_progress = st.empty()
    df = pd.DataFrame(columns=["Time (s)", "Voltage (mV)"])
    progress_bar = placeholder_progress.progress(0.0, text="Running... 0 s")

    try:
        run_task = asyncio.create_task(ocp.run(duration=duration))
        loop = asyncio.get_running_loop()
        start_time = loop.time()
        while not run_task.done():
            elapsed = min(loop.time() - start_time, duration)
            progress_bar.progress(
                min(elapsed / duration, 1.0),
                text=f"Running... {elapsed:.1f} s / {duration} s",
            )
            await asyncio.sleep(0.25)

        data = await run_task
        df = pd.DataFrame(data, columns=["Time (s)", "Voltage (mV)"])
        if df.empty:
            raise RuntimeError("No OCP data was received from the device.")
        fig = px.line(df, x="Time (s)", y="Voltage (mV)", title="OCP")
        placeholder_plot.plotly_chart(fig, use_container_width=True, key="ocp_live_plot_final")
        progress_bar.progress(1.0, text="Complete")
    finally:
        placeholder_plot.empty()
        placeholder_progress.empty()

    return df


# -----------------------------
# Layout
# -----------------------------
cols = st.columns([1, 2], gap="small")

# -----------------------------
# Left – Settings
# -----------------------------
with cols[0].container(border=True):
    st.subheader("Experiment Settings")

    sampling_interval = st.number_input(
        "Sampling Interval (s)", min_value=0.1, value=1.0, step=0.1
    )
    processing_interval = st.number_input(
        "Processing Interval (s)", min_value=0.1, value=1.0, step=0.1
    )
    channel = st.selectbox("Channel", options=[0, 1, 2, 3])
    duration = st.number_input(
        "Duration (s)", min_value=1, max_value=3600, value=30
    )

    exp_name_input = st.text_input(
        "Experiment Name (optional)",
        value=f"OCP Experiment {len(st.session_state.ocp_experiments) + 1}"
    )

    run_button = st.button(
        "Run Experiment",
        use_container_width=True,
        disabled=st.session_state.ocp_running,
    )

    st.divider()
    if st.button("Reset Device Connection", use_container_width=True):
        run_async(reset_device())
        st.success("Device connection reset.")


# -----------------------------
# Right – Results
# -----------------------------
with cols[1].container(border=True):
    st.subheader("Results")

    # Run experiment
    if run_button and not st.session_state.ocp_running:
        st.session_state.ocp_running = True
        status = st.empty()

        try:
            status.info("Running OCP experiment...")
            df = run_async(
                run_ocp_experiment(
                    sampling_interval,
                    processing_interval,
                    channel,
                    duration,
                )
            )

            exp_name = exp_name_input.strip() or f"OCP Experiment {len(st.session_state.ocp_experiments) + 1}"
            st.session_state.ocp_experiments[exp_name] = {
                "data": df,
                "meta": {
                    "Date": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "Sampling Interval (s)": sampling_interval,
                    "Processing Interval (s)": processing_interval,
                    "Channel": channel,
                    "Duration (s)": duration,
                    "Notes": "",
                },
            }

            status.success("Experiment completed successfully!")

        except Exception as e:
            status.error(f"Error: {e}")

        finally:
            st.session_state.ocp_running = False

    # Display experiments
    if st.session_state.ocp_experiments:
        exp_names = list(st.session_state.ocp_experiments.keys())
        selected_exps = st.multiselect("Select experiments", exp_names, default=exp_names)

        display_mode = st.radio("Display mode", ["Plot", "Table"], horizontal=True)

        if display_mode == "Plot":
            fig = px.line(title="OCP Results")
            for name in selected_exps:
                df = st.session_state.ocp_experiments[name]["data"]
                fig.add_scatter(x=df["Time (s)"], y=df["Voltage (mV)"], name=name)
            st.plotly_chart(fig, use_container_width=True, key="ocp_results_plot")
        else:
            combined_df = pd.concat(
                [
                    data["data"].assign(Experiment=name)
                    for name, data in st.session_state.ocp_experiments.items()
                    if name in selected_exps
                ],
                ignore_index=True,
            )
            st.dataframe(combined_df, use_container_width=True)

        # Notes
        st.divider()
        note_exp = st.selectbox("Edit notes", exp_names)
        note = st.text_area(
            "Notes",
            st.session_state.ocp_experiments[note_exp]["meta"]["Notes"],
            height=150,
        )
        if st.button("Save Notes", use_container_width=True):
            st.session_state.ocp_experiments[note_exp]["meta"]["Notes"] = note
            st.success("Notes saved.")

        # Downloads
        st.divider()
        st.subheader("Download Data")

        selected_data = []
        for name in selected_exps:
            entry = st.session_state.ocp_experiments[name]
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
                    "OCP_results.csv",
                    "text/csv",
                    use_container_width=True,
                )
            with col2:
                st.download_button(
                    "Download Excel",
                    excel_buffer.getvalue(),
                    "OCP_results.xlsx",
                    "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
                    use_container_width=True,
                )
