from types import SimpleNamespace
from time import monotonic

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from px4_msgs.msg import VehicleStatus
from px4_uavcup_control.landing.capture_clock import CaptureClock
from px4_uavcup_simulation.landing_readiness import (
    _diagnostic_values, _report,
)


def test_diagnostic_values_extract_first_status_entry():
    message = DiagnosticArray()
    status = DiagnosticStatus()
    status.values = [
        KeyValue(key='target_visible', value='true'),
        KeyValue(key='target_marker_id', value='0'),
    ]
    message.status = [status]

    assert _diagnostic_values(message) == {
        'target_visible': 'true',
        'target_marker_id': '0',
    }


def test_diagnostic_values_handles_empty_status():
    assert _diagnostic_values(DiagnosticArray()) == {}


def test_takeover_requires_current_capture_even_when_marker_is_visible():
    aruco = DiagnosticArray()
    aruco.status = [DiagnosticStatus(values=[
        KeyValue(key='target_visible', value='true'),
        KeyValue(key='target_marker_id', value='0'),
    ])]
    aruco.header.stamp.sec = 692
    node = SimpleNamespace(
        vehicle_status=VehicleStatus(
            pre_flight_checks_pass=True,
            arming_state=VehicleStatus.ARMING_STATE_ARMED),
        local_position=SimpleNamespace(xy_valid=True, z_valid=True),
        attitude=SimpleNamespace(q=[1.0, 0.0, 0.0, 0.0]),
        land_detected=SimpleNamespace(landed=False),
        aruco_status=aruco, landing_status=DiagnosticArray(),
        capture_clock=CaptureClock(), base_messages_ready=lambda: True,
    )
    node.capture_clock.update(891_000_000_000, monotonic())
    assert not _report(node, 'takeover')
    aruco.header.stamp.sec = 891
    assert _report(node, 'takeover')
