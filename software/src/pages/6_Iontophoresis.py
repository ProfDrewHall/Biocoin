import asyncio
import streamlit as st
import pandas as pd
import datetime
from io import BytesIO
import plotly.express as px

from gui_backend import require_backend_runtime

st.set_page_config(
    page_title="Iontophoresis",
    page_icon="⚡",
    layout="wide",
)

st.title("Iontophoresis")
st.caption("Constant-current iontophoresis protocol")
require_backend_runtime()

from biocoin.device import BiocoinDevice
from biocoin.techniques.iontophoresis import Iontophoresis

# ----------------------------
# Session state for async + device
# ----------------------------
if "loop" not in st.session_state:
    st.session_state.loop = asyncio.new_event_loop()
    asyncio.set_event_loop(st.session_state.loop)

if "device" not in st.session_state:
    st.session_state.device = BiocoinDevice()
    st.session_state.device_connected = False

if "ionto_running" not in st.session_state:
    st.session_state.ionto_running = False

if "ionto_experiments" not in st.session_state:
    st.session_state.ionto_experiments = {}


def run_async(coro):
    return st.session_state.loop.run_until_complete(coro)


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


# ----------------------------
# Async iontophoresis experiment
# ----------------------------
async def run_ionto(
    current_monitor_interval,
    stim_current,
    safety_threshold,
    duration,
):
    device = await get_device()
    ionto = Iontophoresis(device)
    await ionto.configure(
        current_monitor_interval=current_monitor_interval,
        stim_current=stim_current,
        current_safety_threshold=safety_threshold,
    )

    plot_placeholder = st.empty()
    progress_placeholder = st.empty()
    progress_bar = progress_placeholder.progress(0.0, text="Running... 0 s")

    df = pd.DataFrame(columns=["Time (s)", "Stim Current (µA)"])
    render_count = 0

    def draw_live_plot(elapsed):
        nonlocal render_count
        render_count += 1
        live_df = pd.DataFrame(
            [[0.0, stim_current], [elapsed, stim_current]],
            columns=["Time (s)", "Stim Current (µA)"],
        )
        fig = px.line(live_df, x="Time (s)", y="Stim Current (µA)", title="Live Iontophoresis Protocol")
        plot_placeholder.plotly_chart(fig, use_container_width=True, key=f"ionto_live_plot_{render_count}")

    try:
        await ionto.clear_queue()
        await device.write_ctrl_command(ionto.Command.START)
        loop = asyncio.get_running_loop()
        start_time = loop.time()
        try:
            while True:
                elapsed = min(loop.time() - start_time, duration)
                draw_live_plot(elapsed)
                progress_bar.progress(
                    min(elapsed / duration, 1.0),
                    text=f"Running... {elapsed:.1f} s / {duration} s",
                )
                if elapsed >= duration:
                    break
                await asyncio.sleep(min(0.25, duration - elapsed))
        finally:
            await device.write_ctrl_command(ionto.Command.STOP)

        df = pd.DataFrame(
            [[0.0, stim_current], [duration, stim_current]],
            columns=["Time (s)", "Stim Current (µA)"],
        )
        progress_bar.progress(1.0, text="Complete")
    finally:
        plot_placeholder.empty()
        progress_placeholder.empty()

    return df


# ----------------------------
# Page layout
# ----------------------------
cols = st.columns([1, 2], gap="small")

# LEFT: Settings
with cols[0].container(border=True):
    st.subheader("Experiment Settings")

    current_monitor_interval = st.number_input("Current Monitor Interval (s)", 0.1, value=1.0)
    stim_current = st.number_input("Stimulation Current (µA)", 1.0, 3000.0, value=100.0)
    safety_threshold = st.number_input("Safety Threshold (µA)", 1.0, 3000.0, value=500.0)
    duration = st.number_input("Duration (s)", 1, 3600, value=60)

    exp_name_input = st.text_input(
        "Experiment Name (optional)",
        value=f"Iontophoresis {len(st.session_state.ionto_experiments) + 1}",
    )

    run_button = st.button(
        "Run Experiment",
        use_container_width=True,
        disabled=st.session_state.ionto_running,
    )

    st.divider()
    if st.button("Reset Device Connection", use_container_width=True):
        run_async(reset_device())
        st.success("Device connection reset.")


# RIGHT: Results
with cols[1].container(border=True):
    st.subheader("Results")

    # Run experiment
    if run_button and not st.session_state.ionto_running:
        st.session_state.ionto_running = True
        status = st.empty()

        try:
            status.info("Running iontophoresis experiment...")
            df = run_async(
                run_ionto(
                    current_monitor_interval,
                    stim_current,
                    safety_threshold,
                    duration,
                )
            )

            exp_name = exp_name_input.strip() or f"Iontophoresis {len(st.session_state.ionto_experiments) + 1}"
            st.session_state.ionto_experiments[exp_name] = {
                "data": df,
                "meta": {
                    "Date": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "Stimulation Current (µA)": stim_current,
                    "Safety Threshold (µA)": safety_threshold,
                    "Monitor Interval (s)": current_monitor_interval,
                    "Duration (s)": duration,
                    "Notes": "",
                },
            }

            status.success("Experiment completed successfully!")

        except Exception as e:
            status.error(f"Error: {e}")

        finally:
            st.session_state.ionto_running = False

    # Display experiments
    if st.session_state.ionto_experiments:
        exp_names = list(st.session_state.ionto_experiments.keys())
        selected_exps = st.multiselect("Select experiments", exp_names, default=exp_names)

        # Plot / Table toggle
        display_mode = st.radio("Display mode", ["Plot", "Table"], horizontal=True)

        if display_mode == "Plot":
            fig = px.line(title="Iontophoresis Results")
            for exp_name in selected_exps:
                df = st.session_state.ionto_experiments[exp_name]["data"]
                fig.add_scatter(x=df["Time (s)"], y=df["Stim Current (µA)"], mode="lines", name=exp_name)
            st.plotly_chart(fig, use_container_width=True, key="ionto_results_plot")
        else:
            combined_df = pd.concat(
                [v["data"].assign(Experiment=name) for name, v in st.session_state.ionto_experiments.items() if name in selected_exps],
                ignore_index=True,
            )
            st.dataframe(combined_df, use_container_width=True)

        # Notes
        st.divider()
        note_exp = st.selectbox("Edit notes", exp_names)
        note = st.text_area(
            "Notes",
            st.session_state.ionto_experiments[note_exp]["meta"]["Notes"],
            height=150,
        )
        if st.button("Save Notes", use_container_width=True):
            st.session_state.ionto_experiments[note_exp]["meta"]["Notes"] = note
            st.success("Notes saved.")

        # Downloads
        st.divider()
        st.subheader("Download Data")
        selected_data = []
        for name in selected_exps:
            entry = st.session_state.ionto_experiments[name]
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
                    "iontophoresis_results.csv",
                    "text/csv",
                    use_container_width=True,
                )
            with col2:
                st.download_button(
                    "Download Excel",
                    excel_buffer.getvalue(),
                    "iontophoresis_results.xlsx",
                    "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
                    use_container_width=True,
                )
