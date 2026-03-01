from unittest.mock import AsyncMock

import pytest


class TechniqueDeviceStub:
    """Reusable async-mock device for technique unit tests."""

    def __init__(self) -> None:
        self.start_notify = AsyncMock()
        self.stop_notify = AsyncMock()
        self.write_ctrl_command = AsyncMock()
        self.write_technique_config = AsyncMock()
        self.client = None


@pytest.fixture
def technique_device() -> TechniqueDeviceStub:
    """Provide a fresh technique-capable device stub."""
    return TechniqueDeviceStub()


@pytest.fixture
def minimal_device() -> object:
    """Provide a bare object for tests that do not use device methods."""
    return object()
