"""
Shared validation helpers for electrochemical technique configuration.
"""

ELECTROCHEM_POTENTIAL_MIN_MV = -2200.0
ELECTROCHEM_POTENTIAL_MAX_MV = 2200.0
ELECTROCHEM_VERTEX_SPAN_MAX_MV = 2200.0
ELECTROCHEM_MAX_CURRENT_UA = 3000.0
ELECTROCHEM_CHANNELS = {0, 1, 2, 3}

CA_MAX_CURRENT_UA = 10_000.0
CA_PULSE_POTENTIAL_MIN_MV = -1000.0
CA_PULSE_POTENTIAL_MAX_MV = 1000.0

PULSE_TIMING_MIN_MS = 3.0
PULSE_TIMING_MAX_MS = 300000.0

CV_E_STEP_MIN_MV = 0.537
CV_E_STEP_MAX_MV = 2200.0

IMP_E_AC_MIN_MV = 0.0
IMP_E_AC_MAX_MV = 2200.0

CV_DAC_FULL_SCALE_RANGE_MV = 2400.0 - 200.0
CV_DAC_RESOLUTION_BITS = 12
CV_DAC_LSB_MV = CV_DAC_FULL_SCALE_RANGE_MV / (2**CV_DAC_RESOLUTION_BITS - 1)


def validate_processing_interval_positive(processing_interval: float) -> None:
    """
    Validate that a processing interval is strictly positive.

    Parameters:
        - processing_interval (float): Interval in seconds.
    Returns:
        - None
    """
    if processing_interval <= 0:
        raise ValueError('processing_interval must be > 0')


def validate_max_current_uA(max_current: float) -> None:
    """
    Validate max current against standard electrochemical bounds.

    Parameters:
        - max_current (float): Current in microamps.
    Returns:
        - None
    """
    if not (0 < max_current <= ELECTROCHEM_MAX_CURRENT_UA):
        raise ValueError('max_current must be > 0 and <= 3000 uA')


def validate_channel_0_to_3(channel: int) -> None:
    """
    Validate channel is in the 0-3 electrochemical channel set.

    Parameters:
        - channel (int): Logical channel index.
    Returns:
        - None
    """
    if channel not in ELECTROCHEM_CHANNELS:
        raise ValueError('channel must be an integer between 0 and 3')


def validate_potential_range(name: str, value: float) -> None:
    """
    Validate a potential is within the electrochemical potential window.

    Parameters:
        - name (str): Parameter name for error reporting.
        - value (float): Potential in millivolts.
    Returns:
        - None
    """
    if not (ELECTROCHEM_POTENTIAL_MIN_MV <= value <= ELECTROCHEM_POTENTIAL_MAX_MV):
        raise ValueError(f'{name} must be between -2200 and +2200 mV')


