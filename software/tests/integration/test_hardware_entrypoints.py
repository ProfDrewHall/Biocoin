import pytest

from tests.integration.CA_2E_validation import build_parser as build_ca_parser
from tests.integration.CA_2E_validation import run_test as run_ca_test
from tests.integration.CV_2E_validation import build_parser as build_cv_parser
from tests.integration.CV_2E_validation import run_test as run_cv_test
from tests.integration.DPV_2E_validation import build_parser as build_dpv_parser
from tests.integration.DPV_2E_validation import run_test as run_dpv_test
from tests.integration.SWV_2E_validation import build_parser as build_swv_parser
from tests.integration.SWV_2E_validation import run_test as run_swv_test


@pytest.mark.hardware
@pytest.mark.asyncio
async def test_ca_2e_validation_default() -> None:
    await run_ca_test(build_ca_parser().parse_args([]))


@pytest.mark.hardware
@pytest.mark.asyncio
async def test_cv_2e_validation_default() -> None:
    await run_cv_test(build_cv_parser().parse_args([]))


@pytest.mark.hardware
@pytest.mark.asyncio
async def test_dpv_2e_validation_default() -> None:
    await run_dpv_test(build_dpv_parser().parse_args([]))


@pytest.mark.hardware
@pytest.mark.asyncio
async def test_swv_2e_validation_default() -> None:
    await run_swv_test(build_swv_parser().parse_args([]))
