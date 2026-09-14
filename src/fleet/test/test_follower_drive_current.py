"""fleet status가 롤러가 아닌 주행 모터 전류만 보고하는지 확인."""
import json

import pytest
import rclpy

from fleet.follower import Follower


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = Follower()
    yield n
    n.destroy_node()


def test_current_ignores_roller_and_stale_drive_samples(node):
    node.motor = {
        'drive_motor_ids': [1, 2],
        'motors': {
            '1': {'current_a': 0.4, 'current_fresh': True},
            '2': {'current_a': 2.0, 'current_fresh': False},
            '9': {'current_a': 9.0, 'current_fresh': True},
        },
    }
    assert node._current() == pytest.approx(0.4)


def test_current_requires_explicit_drive_motor_ids(node):
    node.motor = {
        'motors': {'9': {'current_a': 9.0, 'current_fresh': True}},
    }
    assert node._current() is None


def test_status_publishes_explicit_drive_current_field(node):
    node.motor = {
        'drive_motor_ids': [1, 2],
        'motors': {
            '1': {'current_a': -0.3, 'current_fresh': True},
            '2': {'current_a': 0.5, 'current_fresh': True},
            '9': {'current_a': 8.0, 'current_fresh': True},
        },
    }
    published = []
    node.status_pub.publish = published.append
    node._publish_status()
    payload = json.loads(published[-1].data)
    assert payload['drive_current_a'] == pytest.approx(0.5)
    assert payload['current_a'] == pytest.approx(0.5)
