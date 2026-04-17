import streamlit as st

st.set_page_config(page_title="BioCoin GUI", page_icon="🧪", layout="wide")

st.title("BioCoin GUI")
st.markdown("""
Welcome to the **BioCoin Graphical User Interface (GUI)** 🧪  

Use the sidebar to navigate between electrochemical test techniques:
- ChronoAmperometry (CA)
- Differential Pulse Voltammetry (DPV)
- Cyclic Voltammetry (CV)
- and more coming soon!

Each page allows you to configure, run, and export results for that test.
""")
