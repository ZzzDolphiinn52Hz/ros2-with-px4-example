from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from px4_uavcup_simulation.landing_readiness import _diagnostic_values


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
