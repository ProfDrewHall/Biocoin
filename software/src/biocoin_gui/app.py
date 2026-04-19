import asyncio
from pathlib import Path

import streamlit as st

from biocoin.device import DEVICE_NAME, UUID_DEVICE_SERVICE, BiocoinDevice
from biocoin.utils.ble_util import list_connectable_devices_with_service
from biocoin_gui.runtime import PAGE_METADATA, ensure_session_state

st.set_page_config(page_title='Biocoin GUI', page_icon=':material/science:', layout='wide')

ensure_session_state(
    {
        'connected': False,
        'connection_error': '',
        'ble_devices': [],
        'selected_device': None,
        'biocoin_device': None,
        'device_name': None,
        'initial_device_scan_done': False,
    }
)
if 'ble_loop' not in st.session_state or st.session_state.ble_loop.is_closed():
    st.session_state.ble_loop = asyncio.new_event_loop()


TECHNIQUE_PAGE_ORDER = ['ca', 'dpv', 'cv', 'ocp', 'imp', 'ip', 'temp', 'swv']


def run_ble_coro(coro):
    return st.session_state.ble_loop.run_until_complete(coro)


def refresh_device_list() -> None:
    st.session_state.connection_error = ''
    try:
        with st.spinner('Scanning for connectable Biocoin devices...'):
            devices = run_ble_coro(list_connectable_devices_with_service(UUID_DEVICE_SERVICE, name=DEVICE_NAME))
        st.session_state.ble_devices = devices
        options = [f'{name} ({address})' for name, address, _ in devices]
        if options:
            if st.session_state.selected_device not in options:
                st.session_state.selected_device = options[0]
        else:
            st.session_state.selected_device = None
    except Exception as exc:
        st.session_state.ble_devices = []
        st.session_state.selected_device = None
        st.session_state.connection_error = str(exc)


def connect_to_device() -> None:
    st.session_state.connection_error = ''
    if not st.session_state.selected_device:
        st.session_state.connection_error = 'No device selected.'
        return

    selected = next(
        (
            (name, address)
            for name, address, _ in st.session_state.ble_devices
            if f'{name} ({address})' == st.session_state.selected_device
        ),
        None,
    )
    if selected is None:
        st.session_state.connection_error = 'Selected device is no longer available. Refresh and try again.'
        return

    device_name, device_address = selected

    try:
        biocoin_device = BiocoinDevice()
        with st.spinner(f'Connecting to {device_name}...'):
            run_ble_coro(biocoin_device.connect(name=device_name, address=device_address))
        st.session_state.biocoin_device = biocoin_device
        st.session_state.connected = True
        st.session_state.device_name = device_name
        st.rerun()
    except Exception as exc:
        st.session_state.connected = False
        st.session_state.connection_error = str(exc)
        st.session_state.biocoin_device = None


def disconnect_device() -> None:
    if st.session_state.biocoin_device is None:
        st.session_state.connected = False
        st.session_state.device_name = None
        return

    try:
        run_ble_coro(st.session_state.biocoin_device.disconnect())
    except Exception as exc:
        st.session_state.connection_error = f'Failed to disconnect cleanly: {exc}'
    finally:
        st.session_state.connected = False
        st.session_state.device_name = None
        st.session_state.biocoin_device = None
        st.rerun()


def get_technique_overview_lines() -> list[str]:
    return [f'- {PAGE_METADATA[key].title}' for key in TECHNIQUE_PAGE_ORDER]


def build_pages(pages_dir: Path) -> tuple[st.Page, dict[str, st.Page]]:
    home_page = st.Page(render_home, title='Home', icon=':material/home:')
    technique_pages = {
        'ca': st.Page(str(pages_dir / 'ca.py'), title=PAGE_METADATA['ca'].title, icon=PAGE_METADATA['ca'].icon),
        'dpv': st.Page(str(pages_dir / 'dpv.py'), title=PAGE_METADATA['dpv'].title, icon=PAGE_METADATA['dpv'].icon),
        'cv': st.Page(str(pages_dir / 'cv.py'), title=PAGE_METADATA['cv'].title, icon=PAGE_METADATA['cv'].icon),
        'ocp': st.Page(str(pages_dir / 'ocp.py'), title=PAGE_METADATA['ocp'].title, icon=PAGE_METADATA['ocp'].icon),
        'imp': st.Page(str(pages_dir / 'imp.py'), title=PAGE_METADATA['imp'].title, icon=PAGE_METADATA['imp'].icon),
        'ip': st.Page(str(pages_dir / 'ip.py'), title=PAGE_METADATA['ip'].title, icon=PAGE_METADATA['ip'].icon),
        'temp': st.Page(
            str(pages_dir / 'temp.py'),
            title=PAGE_METADATA['temp'].title,
            icon=PAGE_METADATA['temp'].icon,
        ),
        'swv': st.Page(str(pages_dir / 'swv.py'), title=PAGE_METADATA['swv'].title, icon=PAGE_METADATA['swv'].icon),
    }
    return home_page, technique_pages


def render_sidebar_connection_panel() -> None:
    with st.sidebar:
        st.markdown('### Device Connection')
        if st.session_state.connected and st.session_state.device_name:
            st.success(f'Connected\n\n{st.session_state.device_name}')
        else:
            st.info('Disconnected')

        with st.expander('Connection Controls', expanded=not st.session_state.connected):
            device_options = [f'{name} ({address})' for name, address, _ in st.session_state.ble_devices]
            if device_options:
                selectbox_options = device_options
            elif st.session_state.initial_device_scan_done:
                selectbox_options = ['No compatible devices found']
            else:
                selectbox_options = ['Scanning for devices...']
            st.session_state.selected_device = st.selectbox(
                'Select a BLE device',
                selectbox_options,
                index=selectbox_options.index(st.session_state.selected_device)
                if st.session_state.selected_device in selectbox_options
                else 0,
                disabled=st.session_state.connected or not device_options,
            )

            refresh_clicked = st.button('Refresh Devices', width='stretch', disabled=st.session_state.connected)
            if refresh_clicked:
                refresh_device_list()
                st.rerun()

            if st.session_state.connected:
                disconnect_clicked = st.button('Disconnect', width='stretch')
                if disconnect_clicked:
                    disconnect_device()
            else:
                connect_clicked = st.button('Connect', width='stretch', disabled=not st.session_state.ble_devices)
                if connect_clicked:
                    connect_to_device()

        if st.session_state.connection_error:
            st.caption(st.session_state.connection_error)

        st.divider()


def maybe_run_initial_device_scan() -> None:
    if st.session_state.initial_device_scan_done:
        return

    st.session_state.initial_device_scan_done = True
    refresh_device_list()
    st.rerun()


def render_sidebar_navigation(home_page: st.Page, technique_pages: dict[str, st.Page]) -> None:
    with st.sidebar:
        st.markdown('### Navigation')
        st.page_link(home_page, width='stretch')
        st.caption('Techniques')
        for key in TECHNIQUE_PAGE_ORDER:
            st.page_link(technique_pages[key], width='stretch')


def render_home() -> None:
    st.title('Biocoin GUI')
    technique_lines = '\n'.join(get_technique_overview_lines())
    st.markdown(f"""
Welcome to the **Biocoin Graphical User Interface (GUI)**.

Use the sidebar to navigate between electrochemical techniques:
{technique_lines}

Each page allows you to configure, run, and export results for that test.
""")

    if st.session_state.connected:
        st.success(f'Connected to {st.session_state.device_name}. Use the sidebar to manage the device connection.')
    else:
        st.info('Use the sidebar to select and connect to a Biocoin device before running a technique.')

    st.subheader('Getting Started')
    st.markdown("""
1. Connect to a compatible Biocoin device from the sidebar.
2. Choose a technique page from the navigation menu.
3. Configure the experiment, run it, and export the results.
""")


def main() -> None:
    pages_dir = Path(__file__).with_name('pages')
    home_page, technique_pages = build_pages(pages_dir)
    render_sidebar_connection_panel()
    render_sidebar_navigation(home_page, technique_pages)
    navigation = st.navigation(
        {
            'App': [home_page],
            'Techniques': [technique_pages[key] for key in TECHNIQUE_PAGE_ORDER],
        },
        position='hidden',
    )
    maybe_run_initial_device_scan()
    navigation.run()


if __name__ == '__main__':
    main()
