# `biocoin_gui`

Streamlit-based graphical interface for the Biocoin control software.

## Layout

- `app.py`: Main Streamlit entrypoint and navigation setup.
- `runtime.py`: Shared GUI helpers for session state, background experiment execution, and exports.
- `pages/`: Technique-specific experiment pages.

## Running The GUI

From the repository root:

```bash
uv run --python 3.13 streamlit run src/biocoin_gui/app.py
```

## TODO

- Periodically check that the device connection is still active and update the GUI state when it drops.
- Block other experiments from starting until the current experiment is complete.
