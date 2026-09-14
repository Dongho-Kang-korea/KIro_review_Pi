"""fleet/protocol.py — v1.2 변경(TODO.md 18번): turn/couple 액션,
CMD_TIMEOUT_S 2.5초.
"""
import pytest

from fleet import protocol


def test_turn_and_couple_are_known_actions():
    assert 'turn' in protocol.ACTIONS
    assert 'couple' in protocol.ACTIONS


def test_cmd_timeout_is_2_5s():
    assert protocol.CMD_TIMEOUT_S == 2.5


def test_validate_command_accepts_turn_with_angle_deg():
    result = protocol.validate_command(
        {'t': 1.0, 'action': 'turn', 'seq': 1, 'angle_deg': 180.0})
    assert result['action'] == 'turn'
    assert result['angle_deg'] == 180.0


def test_validate_command_rejects_turn_without_angle_deg():
    with pytest.raises(protocol.ProtocolError):
        protocol.validate_command({'t': 1.0, 'action': 'turn', 'seq': 1})


def test_validate_command_rejects_turn_angle_out_of_range():
    with pytest.raises(protocol.ProtocolError):
        protocol.validate_command(
            {'t': 1.0, 'action': 'turn', 'seq': 1, 'angle_deg': 720.0})


def test_validate_command_accepts_couple_without_extra_fields():
    result = protocol.validate_command({'t': 1.0, 'action': 'couple', 'seq': 2})
    assert result['action'] == 'couple'
