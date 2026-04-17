import asyncio
import streamlit as st
import pandas as pd
import datetime
from io import BytesIO
import plotly.graph_objects as go

from gui_backend import require_backend_runtime

# -----------------------------
# Streamlit config
# -----------------------------
st.set_page_config(
    page_title="Impedance Spectroscopy",
    page_icon="📐",
    layout="wide",
)
st.title("Impedance Spectroscopy (IMP)")
st.caption("Measure impedance magnitude and phase in real time.")
require_backend_runtime()

from biocoin.device import BiocoinDevice
from biocoin.techniques.impedance import Impedance

# -----------------------------
# Session-level async + device
# -----------------------------
if "loop" not in st.session_state:
    st.session_state.loop = asyncio.new_event_loop()
    asyncio.set_event_loop(st.session_state.loop)

if "device" not in st.session_state:
    st.session_state.device = BiocoinDevice()
    st.session_state.device_connected = False

if "imp_running" not in st.session_state:
    st.session_state.imp_running = False

if "imp_experiments" not in st.session_state:
    st.session_state.imp_experiments = {}


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
# Async IMP runner
# -----------------------------
async def run_imp_experiment(
    sampling_interval,
    processing_interval,
    IMP_4wire,
    AC_coupled,
    max_current,
    E_ac,
    frequency,
    duration,
):
    device = await get_device()
    imp = Impedance(device)
    await imp.configure(
        sampling_interval=sampling_interval,
        processing_interval=processing_interval,
        IMP_4wire=IMP_4wire,
        AC_coupled=AC_coupled,
        max_current=max_current,
        E_ac=E_ac,
        frequency=frequency,
    )

    plot_placeholder = st.empty()
    progress_placeholder = st.empty()
    df = pd.DataFrame(columns=["Index", "Magnitude (Ohm)", "Phase (deg)"])
    progress_bar = progress_placeholder.progress(0.0, text="Running... 0 s")

    try:
        run_task = asyncio.create_task(imp.run(duration=duration))
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
        df = pd.DataFrame(list(data), columns=["Magnitude (Ohm)", "Phase (deg)"])
        df.insert(0, "Index", range(len(df)))
        if df.empty:
            raise RuntimeError("No impedance data was received from the device.")

        fig = go.Figure()
        fig.add_trace(go.Scatter(x=df["Index"], y=df["Magnitude (Ohm)"], name="Magnitude (Ohm)", yaxis="y1"))
        fig.add_trace(go.Scatter(x=df["Index"], y=df["Phase (deg)"], name="Phase (deg)", yaxis="y2"))
        fig.update_layout(
            title="Impedance Measurement",
            xaxis_title="Sample Index",
            yaxis=dict(title="Magnitude (Ohm)"),
            yaxis2=dict(title="Phase (deg)", overlaying="y", side="right"),
        )
        plot_placeholder.plotly_chart(fig, use_container_width=True, key="imp_live_plot_final")
        progress_bar.progress(1.0, text="Complete")
    finally:
        plot_placeholder.empty()
        progress_placeholder.empty()

    return df


# -----------------------------
# Layout
# -----------------------------
cols = st.columns([1, 2], gap="small")

# ---------------- LEFT: Settings ----------------
with cols[0].container(border=True):
    st.subheader("Experiment Settings")

    sampling_interval = st.number_input("Sampling Interval (s)", 0.01, value=0.1)
    processing_interval = st.number_input("Processing Interval (s)", 0.01, value=0.1)
    IMP_4wire = st.checkbox("4-Wire Measurement", value=True)
    AC_coupled = st.checkbox("AC Coupled", value=True)
    max_current = st.number_input("Max Current (µA)", 1.0, 3000.0, value=1000.0)
    E_ac = st.number_input("AC Amplitude (mV)", 1.0, 2200.0, value=100.0)
    frequency = st.number_input("Frequency (Hz)", 0.1, value=1000.0)
    duration = st.number_input("Duration (s)", 1, 300, value=10)

    exp_name_input = st.text_input(
        "Experiment Name (optional)",
        value=f"IMP Experiment {len(st.session_state.imp_experiments) + 1}"
    )

    run_button = st.button(
        "Run Experiment",
        use_container_width=True,
        disabled=st.session_state.imp_running,
    )

    st.divider()
    if st.button("Reset Device Connection", use_container_width=True):
        run_async(reset_device())
        st.success("Device connection reset.")


# ---------------- RIGHT: Results ----------------
with cols[1].container(border=True):
    st.subheader("Results")

    # Run experiment
    if run_button and not st.session_state.imp_running:
        st.session_state.imp_running = True
        status = st.empty()

        try:
            status.info("Running IMP experiment...")
            df = run_async(
                run_imp_experiment(
                    sampling_interval,
                    processing_interval,
                    IMP_4wire,
                    AC_coupled,
                    max_current,
                    E_ac,
                    frequency,
                    duration,
                )
            )

            exp_name = exp_name_input.strip() or f"IMP Experiment {len(st.session_state.imp_experiments) + 1}"
            st.session_state.imp_experiments[exp_name] = {
                "data": df,
                "meta": {
                    "Date": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "Sampling Interval (s)": sampling_interval,
                    "Processing Interval (s)": processing_interval,
                    "4-Wire": IMP_4wire,
                    "AC Coupled": AC_coupled,
                    "Max Current (µA)": max_current,
                    "E_ac (mV)": E_ac,
                    "Frequency (Hz)": frequency,
                    "Duration (s)": duration,
                    "Notes": "",
                },
            }

            status.success("Experiment completed successfully!")

        except Exception as e:
            status.error(f"Error: {e}")

        finally:
            st.session_state.imp_running = False

    # Display experiments
    if st.session_state.imp_experiments:
        exp_names = list(st.session_state.imp_experiments.keys())
        selected_exps = st.multiselect("Select experiments", exp_names, default=exp_names)

        display_mode = st.radio("Display mode", ["Plot", "Table"], horizontal=True)

        if display_mode == "Plot":
            fig = go.Figure()
            for name in selected_exps:
                df = st.session_state.imp_experiments[name]["data"]
                fig.add_trace(go.Scatter(x=df["Index"], y=df["Magnitude (Ohm)"], name=f"{name} | Mag", yaxis="y1"))
                fig.add_trace(go.Scatter(x=df["Index"], y=df["Phase (deg)"], name=f"{name} | Phase", yaxis="y2"))
            fig.update_layout(
                title="Impedance Results",
                xaxis_title="Sample Index",
                yaxis=dict(title="Magnitude (Ohm)"),
                yaxis2=dict(title="Phase (deg)", overlaying="y", side="right"),
            )
            st.plotly_chart(fig, use_container_width=True, key="imp_results_plot")
        else:
            combined_df = pd.concat(
                [
                    data["data"].assign(Experiment=name)
                    for name, data in st.session_state.imp_experiments.items()
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
            st.session_state.imp_experiments[note_exp]["meta"]["Notes"],
            height=150,
        )
        if st.button("Save Notes", use_container_width=True):
            st.session_state.imp_experiments[note_exp]["meta"]["Notes"] = note
            st.success("Notes saved.")

        # Downloads
        st.divider()
        st.subheader("Download Data")

        selected_data = []
        for name in selected_exps:
            entry = st.session_state.imp_experiments[name]
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
                    "IMP_results.csv",
                    "text/csv",
                    use_container_width=True,
                )
            with col2:
                st.download_button(
                    "Download Excel",
                    excel_buffer.getvalue(),
                    "IMP_results.xlsx",
                    "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
                    use_container_width=True,
                )
