"""결합 전류는 2호기 소유, 적재에는 1호기 기준 파라미터가 없는지 확인."""

import json

import pytest
import rclpy
from mission.common.coupling import Coupling
from mission.summer.cargo_load import CargoLoad


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def motor_status(threshold):
    return json.dumps({'can_connected': True,
                       'contact_current_threshold_a': threshold,
                       'motors': {}})


@pytest.fixture
def coupling():
    node = Coupling()
    yield node
    node.destroy_node()


@pytest.fixture
def cargo():
    node = CargoLoad()
    yield node
    node.destroy_node()


def set_coupling_threshold(node, value):
    node.set_parameters([
        rclpy.parameter.Parameter('contact_current_threshold_a',
                                  rclpy.Parameter.Type.DOUBLE, value)])


# ---------- 폴백 ----------

def test_coupling_falls_back_to_motor_value(coupling):
    """자기 값이 0 이면 motor/status 의 공통값을 쓴다 (기존 동작 유지)."""
    set_coupling_threshold(coupling, 0.0)
    coupling._on_motor(type('M', (), {'data': motor_status(1.1)})())
    assert coupling.contact_threshold() == pytest.approx(1.1)


def test_cargo_has_only_unit2_load_current_parameters(cargo):
    assert not cargo.has_parameter('leader_contact_current_threshold_a')
    assert not cargo.has_parameter('unit1_load_high_current_a')
    assert not cargo.has_parameter('unit1_load_low_current_a')
    assert cargo.has_parameter('unit2_load_high_current_a')
    assert cargo.has_parameter('unit2_load_low_current_a')


# ---------- 자기 값이 이긴다 ----------

def test_coupling_own_value_wins(coupling):
    """결합은 빠르게 붙어 전류가 크게 튄다 — 공통값보다 자기 값이 우선."""
    set_coupling_threshold(coupling, 1.75)
    coupling._on_motor(type('M', (), {'data': motor_status(0.9)})())
    assert coupling.contact_threshold() == pytest.approx(1.75)


# ---------- 둘 다 0 이면 판정 안 함 ----------

def _reach_threshold_gate(node):
    """_eligibility() 가 전류 기준 검사까지 도달하도록 앞선 조건을 채운다.

    검사 순서가 active -> locked -> reference -> marker -> **전류 기준** 이라,
    그냥 새 노드로 부르면 reference_wait 에서 먼저 걸린다.
    """
    node.reference = {'x_mm': 0.0, 'y_mm': 0.0, 'z_mm': 300.0,
                      'roll_deg': 0.0, 'pitch_deg': 0.0, 'yaw_deg': 0.0}
    node.marker_stable = True


def test_both_zero_blocks_contact(coupling):
    """실측 전 안전 상태 — 결합 자격 검사가 전류 기준에서 막아야 한다."""
    set_coupling_threshold(coupling, 0.0)
    coupling._on_motor(type('M', (), {'data': motor_status(0.0)})())
    _reach_threshold_gate(coupling)
    assert coupling.contact_threshold() == 0.0
    eligible, reason = coupling._eligibility()
    assert eligible is False
    assert reason == 'current_threshold_unset'


def test_own_value_passes_threshold_gate(coupling):
    """자기 값만 있어도(공통값 0) 전류 기준 검사를 통과해야 한다."""
    set_coupling_threshold(coupling, 1.75)
    coupling._on_motor(type('M', (), {'data': motor_status(0.0)})())
    _reach_threshold_gate(coupling)
    eligible, reason = coupling._eligibility()
    assert reason != 'current_threshold_unset'
