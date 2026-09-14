"""2호기 본체 실행. 모드는 UI 가 런타임에 바꾼다.

    ros2 launch mars_launch robot.launch.py                  # 2호기
    ros2 launch mars_launch robot.launch.py season:=escort   # 계절만 다르게

1호기의 ``bringup/robot.launch.py`` 와 이름을 맞췄다. 두 호기 모두 본체는
``robot.launch.py`` 로 시작한다.

계절
----
2호기는 **여름에만 자체 미션(화물 적재)이 있고 나머지 구간은 전부 추종**이다.
그래서 계절별로 띄우는 노드가 갈리지 않는다 — ``cargo_load`` 는 항상 올려두고
명령이 없으면 가만히 있는다. ``season`` 인자는 UI 표시와 기록용이다.

네임스페이스
------------
모든 노드가 ``/unit2`` 아래에 뜬다. **1호기와 토픽 이름이
겹치기 때문이다** — 양쪽 다 ``cmd_vel`` / ``roller_cmd`` / ``motor/status`` 를
쓰므로, 같은 ``ROS_DOMAIN_ID`` 에 올라오면 1호기 주행 명령이 2호기 모터로
그대로 들어간다.

그래서 노드 코드의 내부 토픽은 전부 **상대 이름**이다. 앞에 슬래시를 붙이면
네임스페이스를 무시하고 루트로 올라가 충돌이 되살아난다. 새 토픽을 추가할
때 반드시 상대 이름으로 쓸 것. 예외는 호기 간 규약 토픽 ``/fleet/*`` 하나다.

모드
----
시작은 항상 ``idle`` 이다. 운용 UI 가 ``drive/mode_cmd`` 로 올린다.

    manual  수동 주행        1호기 없이 동작
    auto    여름 화물 적재    1호기 없이 동작
    follow  1호기 추종        /fleet/state 수신 중일 때만 진입 가능
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SEASONS = ('spring', 'summer', 'autumn', 'winter', 'escort')


def cfg(package, name):
    return os.path.join(get_package_share_directory(package), 'config', name)


def build(context, *args, **kwargs):
    unit = int(LaunchConfiguration('unit').perform(context))
    season = LaunchConfiguration('season').perform(context)
    if unit != 2:
        # 3호기는 설계에서 빠졌다 (TODO 19, 2026-08-31 사용자 확정).
        raise RuntimeError(f'unit 은 2 여야 한다: {unit}')
    if season not in SEASONS:
        raise RuntimeError(
            f'모르는 계절 {season!r}. 가능한 값: {", ".join(SEASONS)}')

    namespace = f'unit{unit}'
    camera_env = {
        'PYTHONPATH': '/usr/local/lib/python3/dist-packages:' +
                      os.environ.get('PYTHONPATH', '')}

    def node(package, executable, config, extra=None, **kwargs):
        parameters = [cfg(*config)]
        if extra:
            parameters.append(extra)
        return Node(package=package, executable=executable, namespace=namespace,
                    output='screen', parameters=parameters, **kwargs)

    # 인수인계: 수치 튜닝은 아래 parameters 가 가리키는 config YAML 에서 하고,
    # 노드 추가/삭제가 필요할 때만 이 목록을 고친다.
    return [
        # 하드웨어
        node('base', 'motor', ('base', 'motor.yaml')),
        node('base', 'camera', ('base', 'camera.yaml'), additional_env=camera_env),
        node('base', 'marker_vision', ('base', 'marker.yaml')),
        node('base', 'imu', ('base', 'imu.yaml')),
        node('base', 'uwb', ('base', 'uwb.yaml')),
        node('base', 'solenoid', ('base', 'solenoid.yaml')),

        # 1호기 통신. 없어도 manual/auto 는 그대로 돈다.
        node('fleet', 'follower', ('fleet', 'follower.yaml'), {'unit': unit}),
        # RC(무선 조종기) 채널 중계 — 수신기는 1호기에만 있고, 1호기가
        # 채널 값을 토픽으로 넘기면 이 노드가 cmd_vel/rc로 바꾼다. 신호가
        # 없어도 안전(그냥 rc 모드에서 안 움직임)이라 항상 띄운다. 토픽
        # 이름/채널 매핑은 전부 임시값 — TODO.md 14번, fleet/rc_bridge.yaml.
        node('fleet', 'rc_bridge', ('fleet', 'rc_bridge.yaml'), {'unit': unit}),

        # 미션 — 추종(모든 구간), 화물 적재(여름), 자동 결합
        node('mission', 'follow_leader', ('mission', 'follow.yaml')),
        node('mission', 'cargo_load', ('mission', 'cargo.yaml')),
        node('mission', 'coupling', ('mission', 'coupling.yaml')),
        node('mission', 'coupled_drive', ('mission', 'coupled.yaml')),

        # 모드 소유자 겸 안전 게이트
        node('drive', 'arbiter', ('drive', 'arbiter.yaml'), {'season': season}),
    ]


def generate_launch_description():
    return LaunchDescription([
        # 1·2호기가 서로 보이려면 모두 ROS_DOMAIN_ID=10 이어야 한다.
        SetEnvironmentVariable('ROS_DOMAIN_ID', '10'),
        DeclareLaunchArgument('unit', default_value='2',
                              description='호기 번호 (2 또는 3). 네임스페이스가 된다'),
        DeclareLaunchArgument('season', default_value='summer',
                              description='|'.join(SEASONS) + ' — 표시용'),
        OpaqueFunction(function=build),
    ])
