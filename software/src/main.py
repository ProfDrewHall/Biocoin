#######################################################################################################################
#
# BioCoin Device Sample Test Script
#
# Provides a standalone entry point for testing and validating the core functionality of the BioCoin device.
# This script exercises BLE connectivity, data acquisition, technique configuration, and parameter handling.
# Intended for use during development, debug, and hardware bring-up.
#
# Written By:
#   - Drew Hall (DrewHall@ucsd.edu)
#   - Risab Sankar (rsankar@ucsd.edu)
#   - Tyler Hack (thack@ucsd.edu)
#
# Revision History:
#   - 28 June 2025: Initial creation for system-level BioCoin testing
#
# Known Issues:
#   - Hardcoded test parameters and UUIDs may require updates for deployment
#   - Assumes BLE peripheral is advertising and responsive before execution
#
#   - Change the device name tested
#
# To Do:
#   - Add ability to set advanced parameters
#
#
#######################################################################################################################
import asyncio
import logging
from pathlib import Path

import pandas as pd

from biocoin.device import BioCoinDevice
from biocoin.techniques import (
    OpenCircuitPotential,
    Temperature,
    Iontophoresis
)
from utils.logging_util import setup_logging

# Setup logging
logger = logging.getLogger(__name__)


async def main():
    setup_logging()  # Initialize logging
    logging.info('BioCoin Test Script')

    device = BioCoinDevice()

    try:
        await device.connect()

        # Get battery level
        battery = await device.get_battery_level()

        # Change the device name
        # await device.change_device_name('BioCoin')

        # Run a CA measurement
        # CA = ChronoAmperometry(device)
        # await CA.configure(
        #     sampling_interval=2,
        #     processing_interval=4.0,
        #     max_current=100.0,
        #     pulse_potential=200.0,
        #     channel=0,
        # )

        # CA_data = await CA.run(duration=20)  # Note -- the 8s does NOT mean you will get 8 points with a long BLE latency!!!
        # logging.info(f'CA Data:\n {CA_data}')

        # logging.info('Saving CA data to CSV file...')
        # df = pd.DataFrame(CA_data, columns=['Time (s)', 'Current (uA)'])
        # output_path = Path('./results/CA_output.csv')
        # output_path.parent.mkdir(parents=True, exist_ok=True)
        # df.to_csv(output_path, index=False)

        # await asyncio.sleep(5)

        # Run a CV measurement
        # CV = CyclicVoltammetry(device)
        # await CV.configure(
        #     processing_interval=1.0,
        #     max_current=100.0,
        #     E_start=-200.0,
        #     E_vertex1=200.0,
        #     E_vertex2=-200.0,
        #     E_step=50.0,
        #     pulse_width=50,
        #     channel=0,
        # )
        # CV_data = await CV.run()
        # logging.info(f'CV Data:\n {CV_data}')

        # logging.info('Saving CV data to CSV file...')
        # df = pd.DataFrame(CV_data, columns=['Voltage (mV)', 'Current (uA)'])
        # output_path = Path('./results/CV_output.csv')
        # output_path.parent.mkdir(parents=True, exist_ok=True)
        # df.to_csv(output_path, index=False)

        # # Run a DPV measurement
        # DPV = DifferentialPulseVoltammetry(device)
        # await DPV.configure(
        #     processing_interval=1.0,
        #     max_current=100.0,
        #     E_start=100.0,
        #     E_stop=200.0,
        #     E_step=50.0,
        #     E_pulse=100.0,
        #     pulse_width=150,
        #     pulse_period=400,
        #     channel=0,
        # )
        # DPV_data = await DPV.run()
        # logging.info(f'DPV Data:\n {DPV_data}')

        # logging.info('Saving DPV data to CSV file...')
        # df = pd.DataFrame(DPV_data, columns=['Voltage (mV)', 'Current (uA)'])
        # output_path = Path('./results/DPV_output.csv')
        # output_path.parent.mkdir(parents=True, exist_ok=True)
        # df.to_csv(output_path, index=False)

        # Run an impedance measurement
        # Imp = Impedance(device)
        # await Imp.configure(
        #     sampling_interval=2,
        #     processing_interval=4.0,
        #     max_current=100.0,
        #     IMP_4wire=True,
        #     AC_coupled=False,
        #     E_ac=10.0,
        #     frequency=100.0)
        # Imp_data = await Imp.run(duration=15)
        # logging.info(f'Imp Data:\n {Imp_data}')

        # logging.info('Saving Imp data to CSV file...')
        # df = pd.DataFrame(Imp_data, columns=['Magnitude (Ohms)', 'Phase (deg)'])
        # output_path = Path('./results/Imp_output.csv')
        # output_path.parent.mkdir(parents=True, exist_ok=True)
        # df.to_csv(output_path, index=False)

        # Run a temp measurement
        # Temp = Temperature(device)
        # await Temp.configure(
        #     sampling_interval=2,
        #     processing_interval=4.0,
        #     channel=0,
        # )

        # Temp_data = await Temp.run(
        #     duration=20
        # )  # Note -- the 8s does NOT mean you will get 8 points with a long BLE latency!!!
        # logging.info(f'Temp Data:\n {Temp_data}')

        # logging.info('Saving Temp data to CSV file...')
        # df = pd.DataFrame(Temp_data, columns=['Time (s)', 'Voltage (mV)'])
        # output_path = Path('./results/Temp_output.csv')
        # output_path.parent.mkdir(parents=True, exist_ok=True)
        # df.to_csv(output_path, index=False)

        # await asyncio.sleep(5)

        # # Run an OCP measurement
        # OCP = OpenCircuitPotential(device)
        # await OCP.configure(
        #     sampling_interval=2,
        #     processing_interval=4.0,
        #     channel=0,
        # )

        # OCP_data = await OCP.run(
        #     duration=20
        # )  # Note -- the 8s does NOT mean you will get 8 points with a long BLE latency!!!
        # logging.info(f'OCP Data:\n {OCP_data}')

        # logging.info('Saving OCP data to CSV file...')
        # df = pd.DataFrame(OCP_data, columns=['Time (s)', 'Voltage (mV)'])
        # output_path = Path('./results/OCP_output.csv')
        # output_path.parent.mkdir(parents=True, exist_ok=True)
        # df.to_csv(output_path, index=False)

        # await asyncio.sleep(5)

        # Start iontopheresis
        iontophoresis = Iontophoresis(device)
        await iontophoresis.configure(
            current_monitor_interval=1.0,
            stim_current=5.0,
            current_safety_threshold=20.0,
        )
        _ = await iontophoresis.run(duration=150, poll_interval=3)
        
        # await asyncio.sleep(5)

    except asyncio.TimeoutError:
        logging.error('Connection timed out. Please ensure the device is powered on and in range.')

    except Exception as e:
        logging.exception(f'{e}')

    finally:
        await device.disconnect()

    input('All done! Press Enter to exit...')


if __name__ == '__main__':
    asyncio.run(main())
