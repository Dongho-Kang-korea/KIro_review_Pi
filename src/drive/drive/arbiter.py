"""주행 명령 하나를 선택하고, 호기 운용 모드를 소유한다.

모드
----
``drive/mode_cmd`` 로 런타임 전환한다. 계절은 launch 인자로 고정이고 모드만 바뀐다.

===========  ==================  =========================  ==========
모드          주행 소스            미션 명령 출처              1호기 필요
===========  ==================  =========================  ==========
``idle``     없음 (0)             없음                        아니오
``manual``   ``cmd_vel/manual``   없음 (인식 노드는 계속 돎)    아니오
``auto``     미션 노드            ``mission/request`` (UI)    아니오
``follow``   (아래 참고)           ``/fleet/cmd/unitN`` (1호기) **예**
``rc``       ``cmd_vel/rc``       없음                        아니오(단, 신호는 1호기 경유)
===========  ==================  =========================  ==========

``follow`` 모드의 주행 소스는 고정이 아니다 — 1호기가 보낸 액션이
``CARGO_ACTIONS``(``approach``/``turn``/``load``/``hold``/``couple``/
``release``)에 속하면 ``cmd_vel/cargo``를, 아니면 ``cmd_vel/follow``를
쓴다. 여름 이동 구간엔 ``follow``인 채로 화물 명령이 오는 경우가 흔해서다
— 예전엔 여기서 무조건 ``follow``를 반환해, ``mission/action``은 정상
갱신되는데 실제 모터는 안 움직이는 버그가 있었다(2026-08-31 수정,
TODO.md 16번). ``turn``/``couple``은 v1.2 프로토콜(TODO.md 18번)에서
추가됐다.

``rc`` 모드는 1호기에만 물린 RC(무선 조종기) 수신기의 채널 값을
``fleet/rc_bridge`` 가 받아 ``cmd_vel/rc`` 로 바꾼 것을 쓴다. ``follow`` 와
달리 진입에 별도 생존 게이트가 없다 — ``manual`` 과 동일하게, 신호가 없으면
아래 ``source_timeout_s`` 일반 안전장치가 그대로 0을 낸다.

미션 명령 다중화
----------------
``cargo_load`` 와 ``follow_leader`` 는 예전에 ``fleet/action`` 만 봤다. 그래서
1호기가 없으면 아무 미션도 시작할 수 없었다. 이제 이 노드가 모드에 따라
**권위 있는 명령 하나를 골라** ``mission/action`` 으로 재발행하고, 미션 노드는
그것만 구독한다.

    mode=follow -> fleet/action (1호기)     -+
    mode=auto   -> mission/request (UI)     -+-> mission/action
    그 외        -> {"action": "idle"}       -+

``mission/request`` 는 "해달라는 요청", ``mission/action`` 은 "확정된 명령"이다.
1호기 쪽 ``/fleet/request`` 와 같은 뜻으로 쓴다.

``mission/action`` 은 **바뀔 때만** 발행한다. 같은 명령을 반복 발행하면
``cargo_load`` 의 단계 전이가 되감긴다 (load 를 다시 받으면 loaded -> loading).
대신 transient-local QoS 라 늦게 뜬 노드도 마지막 명령을 받는다.

추종 모드는 ``/fleet/state`` 가 살아 있을 때만 들어갈 수 있고, 들어간 뒤
``fleet_state_timeout_s`` 동안 끊기면 ``idle`` 로 강등한다.

안전 우선순위는 그대로다: 비상 정지 > 통신 안전 정지 > 소스 시간 초과 > 선택된 명령.
"""
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

MODES = ('idle', 'manual', 'auto', 'follow', 'coupled', 'rc')

# fleet/protocol.py 의 ACTIONS 와 같아야 한다. drive 가 fleet 에 빌드 의존하지
# 않도록 여기 복사해 뒀다 — 한쪽을 고치면 반드시 함께 고칠 것.
ACTIONS = ('idle', 'follow', 'approach', 'turn', 'load', 'hold',
           'couple', 'release', 'stop')
CARGO_ACTIONS = ('approach', 'turn', 'load', 'hold', 'couple', 'release')

# origin 은 '명령이 어디서 왔나' (fleet/ui/mode) 다.
# 주행 후보를 뜻하는 status 의 source 와 헷갈리지 않도록 이름을 나눠 뒀다.
IDLE_ACTION = {'action': 'idle', 'origin': 'mode'}


class Arbiter(Node):
    # 최종 cmd_vel 은 이 노드만 발행하며 비상 정지가 항상 우선한다.
    # 인수인계: SOURCES는 '후보 이름 -> 토픽' 인터페이스다. 토픽 이름을
    # 바꾸면 해당 미션 노드와 함께 바꿔야 한다. 우선순위 규칙은 _source(),
    # timeout 수치는 drive/config/arbiter.yaml에서 관리한다.
    SOURCES = {
        'follow': 'cmd_vel/follow',
        'cargo': 'cmd_vel/cargo',
        'coupling': 'cmd_vel/coupling',
        # 결합 성공 후 1호기와 같은 속도로 굴러가는 협조 주행.
        # coupling 은 결합 '과정', coupled 는 결합 '이후'다.
        'coupled': 'cmd_vel/coupled',
        'manual': 'cmd_vel/manual',
        'rc': 'cmd_vel/rc',
    }

    def __init__(self):
        super().__init__('arbiter')

        # 수치는 drive/config/arbiter.yaml에서 조정한다.
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('source_timeout_s', 0.5)
        # 시작 모드. 안전을 위해 기본은 idle 이고 UI 가 올려준다.
        self.declare_parameter('start_mode', 'idle')
        # 계절은 launch 인자로 고정한다. 여기서는 UI 표시와 기록에만 쓴다.
        self.declare_parameter('season', 'summer')
        # 추종 중 /fleet/state 가 이 시간 끊기면 idle 로 강등한다.
        self.declare_parameter('fleet_state_timeout_s', 2.5)

        self.season = str(self.get_parameter('season').value)
        self.fleet_timeout = float(self.get_parameter('fleet_state_timeout_s').value)

        self.mode = self._initial_mode()
        self.fleet_action = None        # 1호기가 보낸 마지막 명령
        self.ui_request = None          # UI 가 보낸 마지막 미션 요청
        self.published_action = None    # mission/action 으로 마지막에 낸 것
        self.fleet_state_time = None
        self.safety_stop = False
        self.emergency_stop = False
        self.latest = {name: Twist() for name in self.SOURCES}
        self.received = {name: None for name in self.SOURCES}
        self.follow_status = {}
        self.cargo_status = {}
        self.coupling_status = {}
        self.output = Twist()

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'drive/status', 10)
        self.phase_pub = self.create_publisher(String, 'mission/phase', 10)
        # 늦게 뜬 미션 노드도 마지막 명령을 받도록 latched 로 낸다.
        self.action_pub = self.create_publisher(
            String, 'mission/action',
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        for name, topic in self.SOURCES.items():
            self.create_subscription(
                Twist, topic, lambda msg, key=name: self._on_cmd(key, msg), 10)
        self.create_subscription(String, 'fleet/action', self._on_fleet_action, 10)
        self.create_subscription(String, 'mission/request', self._on_ui_request, 10)
        # 규약 토픽이라 네임스페이스 밖 절대 경로다. 1호기 생존 판정에 쓴다.
        self.create_subscription(String, '/fleet/state', self._on_fleet_state, 10)
        self.create_subscription(Bool, 'safety/stop', self._on_safety_stop, 10)
        self.create_subscription(
            Bool, 'safety/emergency_stop', self._on_emergency_stop, 10)
        self.create_subscription(String, 'drive/mode_cmd', self._on_mode, 10)
        self.create_subscription(
            String, 'mission/follow/status', self._on_follow_status, 10)
        self.create_subscription(
            String, 'mission/cargo/status', self._on_cargo_status, 10)
        self.create_subscription(
            String, 'mission/coupling/status', self._on_coupling_status, 10)

        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(0.2, self._publish_status)
        self._publish_action()          # 시작 상태를 한 번 걸어둔다
        self.get_logger().info(
            f'arbiter ready | season={self.season} | mode={self.mode}')

    # ---------- 모드 ----------

    def _initial_mode(self):
        mode = str(self.get_parameter('start_mode').value).strip().lower()
        if mode not in MODES:
            self.get_logger().warn(f'모르는 start_mode {mode!r} -> idle')
            return 'idle'
        if mode == 'follow':
            # 시작 시점에는 1호기 생존을 알 수 없다. UI 가 올리게 둔다.
            self.get_logger().warn('start_mode=follow 는 시작 시 판정할 수 없다 -> idle')
            return 'idle'
        return mode

    def fleet_alive(self):
        return (self.fleet_state_time is not None and
                time.monotonic() - self.fleet_state_time <= self.fleet_timeout)

    def rc_alive(self):
        # follow의 fleet_alive()와 달리 모드 진입을 막는 게이트는 아니다
        # (rc는 manual처럼 신호가 없으면 그냥 0을 낸다) — UI에 링크 상태만
        # 보여주는 용도. source_timeout_s를 그대로 재사용해 별도 파라미터를
        # 안 늘렸다.
        received = self.received.get('rc')
        return (received is not None and time.monotonic() - received <=
                float(self.get_parameter('source_timeout_s').value))

    def _set_mode(self, mode, reason=''):
        if mode == self.mode:
            return
        self.get_logger().info(
            f'MODE: {self.mode} -> {mode}' + (f' ({reason})' if reason else ''))
        self.mode = mode
        # 모드가 바뀌면 남은 미션 요청을 그대로 끌고 가지 않는다.
        if mode in ('idle', 'manual'):
            self.ui_request = None
        self._publish_action()
        self.cmd_pub.publish(Twist())   # 전환 순간 안전 정지 1회

    def _on_mode(self, msg):
        value = str(msg.data).strip().lower()
        if value == 'reset_emergency':
            self.emergency_stop = False
            self.get_logger().info('비상 정지 해제')
            return
        if value not in MODES:
            self.get_logger().warn(f'모르는 mode_cmd {value!r} -> 무시')
            return
        if value in ('follow', 'coupled') and not self.fleet_alive():
            # 1호기가 안 잡히면 추종에 들어가지 않는다. 마커만 보고 따라가면
            # 1호기 상태를 모른 채 움직이게 된다.
            self.get_logger().warn(
                f'1호기 /fleet/state 가 없다 -> {value} 진입 거부')
            return
        self._set_mode(value, '운용자 지시')

    # ---------- 미션 명령 다중화 ----------

    @staticmethod
    def _clean(payload, origin):
        """action 목록과 roller 범위를 검사해 통과한 것만 돌려준다."""
        if not isinstance(payload, dict):
            return None
        action = str(payload.get('action', '')).strip().lower()
        if action not in ACTIONS:
            return None
        result = {'action': action, 'origin': origin}
        if 'seq' in payload:
            result['seq'] = payload['seq']
        if action == 'load':
            roller = payload.get('roller')
            if not isinstance(roller, (int, float)) or isinstance(roller, bool):
                return None
            result['roller'] = max(-1.0, min(1.0, float(roller)))
        if action == 'turn':
            angle = payload.get('angle_deg')
            if not isinstance(angle, (int, float)) or isinstance(angle, bool):
                return None
            angle = float(angle)
            if not -360.0 <= angle <= 360.0:
                return None
            result['angle_deg'] = angle
        return result

    def _on_fleet_action(self, msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self.fleet_action = self._clean(payload, 'fleet')
        if self.mode == 'follow':
            self._publish_action()

    def _on_ui_request(self, msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        request = self._clean(payload, 'ui')
        if request is None:
            self.get_logger().warn(f'미션 요청 형식 오류 -> 버림: {msg.data!r}')
            return
        self.ui_request = request
        if self.mode == 'auto':
            self._publish_action()

    def _authoritative(self):
        if self.mode == 'follow' and self.fleet_action is not None:
            return self.fleet_action
        if self.mode == 'auto' and self.ui_request is not None:
            return self.ui_request
        return dict(IDLE_ACTION)

    def _publish_action(self):
        """바뀌었을 때만 낸다. 같은 명령을 반복하면 단계가 되감긴다."""
        action = self._authoritative()
        if action == self.published_action:
            return
        self.published_action = action
        self.action_pub.publish(String(
            data=json.dumps(dict(action, t=time.time()), separators=(',', ':'))))

    def action(self):
        return (self.published_action or IDLE_ACTION).get('action', 'idle')

    # ---------- 수신 ----------

    def _on_cmd(self, name, msg):
        self.latest[name], self.received[name] = msg, time.monotonic()

    def _on_fleet_state(self, msg):
        self.fleet_state_time = time.monotonic()

    def _on_safety_stop(self, msg): self.safety_stop = bool(msg.data)
    def _on_emergency_stop(self, msg): self.emergency_stop = bool(msg.data)

    def _on_follow_status(self, msg):
        try: self.follow_status = json.loads(msg.data)
        except Exception: pass

    def _on_cargo_status(self, msg):
        try: self.cargo_status = json.loads(msg.data)
        except Exception: pass

    def _on_coupling_status(self, msg):
        try: self.coupling_status = json.loads(msg.data)
        except Exception: pass

    # ---------- 중재 ----------

    def _source(self):
        # 결합은 모드보다 위다. 진행 중이거나 오류면 결합기가 속도를 쥔다.
        # 'locked'(성공)는 빼 둔다 — 넣으면 결합에 성공한 뒤 취소해서 솔레노이드를
        # 풀기 전까지 영영 못 움직인다.
        if self.coupling_status.get('active') or self.coupling_status.get('phase') in (
                'locking', 'error'):
            return 'coupling'
        # 결합 이동은 미션 명령과 무관하게 1호기 속도만 복제한다.
        if self.mode == 'coupled':
            return 'coupled'
        if self.mode == 'manual':
            return 'manual'
        if self.mode == 'rc':
            return 'rc'
        if self.mode == 'follow':
            # follow 모드(1호기 추종 중)에서도 1호기가 화물 명령을 보낼
            # 수 있다(여름 이동 구간 도중 목적지에 도착해 접근/적재를
            # 시작하는 경우). mission/action은 이미 정상 갱신되어
            # cargo_load.py가 단계를 진행하는데, 예전엔 여기서 무조건
            # 'follow'를 반환해 cmd_vel/cargo가 있어도 안 쓰이고 실제
            # 모터는 안 움직이는 버그가 있었다(TODO.md 16번).
            action = self.action()
            return 'cargo' if action in CARGO_ACTIONS else 'follow'
        if self.mode == 'auto':
            action = self.action()
            if action in CARGO_ACTIONS:
                return 'cargo'
            if action == 'follow':
                return 'follow'
        return None

    def _phase(self):
        source = self._source()
        coupling_phase = self.coupling_status.get('phase')
        # 실제 결합 검증까지 끝난 locked는 주행 소스가 cargo/follow로
        # 넘어간 뒤에도 상위 fleet 상태에 계속 유지해서 보고한다.
        if coupling_phase == 'locked':
            return 'locked'
        if source == 'coupling':
            return {
                'align': 'approach', 'contact': 'contact',
                # 솔레노이드 작동 및 후진 당김 검증 중에는 아직 성공이 아니다.
                'locking': 'contact', 'verify_pull': 'contact',
                'error': 'error',
            }.get(coupling_phase, 'idle')
        if source == 'follow': return self.follow_status.get('phase', 'following')
        if source == 'cargo': return self.cargo_status.get('phase', 'idle')
        return 'idle'

    def _tick(self):
        # 추종 중 1호기가 끊기면 스스로 내려온다.
        if self.mode in ('follow', 'coupled') and not self.fleet_alive():
            self._set_mode('idle', '1호기 /fleet/state 두절')

        # 결합이 끝나면 스스로 협조 주행으로 올라간다. 대회는 수동/자동
        # 주행 구간이 나뉘어 있고 자동 구간에서는 사람이 개입할 수 없는데,
        # 예전에는 locked 가 돼도 누군가 drive/mode_cmd 로 coupled 를
        # 쏴 줘야만 협조 주행이 시작돼 그대로 멈춰 섰다(TODO 22번).
        # 수동 조종 중(manual/rc)에는 올리지 않는다 — 사람이 쥔 조종을
        # 뺏으면 안 된다.
        if (self.coupling_status.get('phase') == 'locked' and
                self.mode not in ('manual', 'rc', 'coupled')):
            self._set_mode('coupled', '결합 완료')

        source = self._source()
        output = Twist()
        if not self.safety_stop and not self.emergency_stop and source is not None:
            received = self.received[source]
            if received is not None and time.monotonic() - received <= float(
                    self.get_parameter('source_timeout_s').value):
                output = self.latest[source]
        self.output = output
        self.cmd_pub.publish(output)

    def _publish_status(self):
        phase = self._phase()
        self.phase_pub.publish(String(data=phase))
        self.status_pub.publish(String(data=json.dumps({
            'mode': self.mode, 'season': self.season,
            'action': self.action(), 'source': self._source(), 'phase': phase,
            # 1호기가 잡히는가. UI 는 이 값 하나로 추종 버튼을 켜고 끈다.
            'fleet_link': self.fleet_alive(),
            # 1호기 경유 RC 채널 신호가 살아있는가. rc 모드는 진입을
            # 막지 않으므로(manual과 동일) 이건 순수 표시용이다.
            'rc_link': self.rc_alive(),
            'safety_stop': self.safety_stop,
            'emergency_stop': self.emergency_stop,
            'coupling_phase': self.coupling_status.get('phase'),
            'coupling_eligible': self.coupling_status.get('eligible', False),
            'linear': self.output.linear.x, 'angular': self.output.angular.z,
        })))


def main(args=None):
    rclpy.init(args=args); node = Arbiter()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
