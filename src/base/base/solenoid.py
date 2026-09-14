"""GPIO 릴레이로 솔레노이드를 제어하며 시작·종료 시 항상 안전한
잠금(무통전) 상태로 둔다."""

import sys

import lgpio

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class SolenoidNode(Node):
    def __init__(self):
        super().__init__('solenoid')

        # 릴레이 핀과 동작 레벨은 base/config/solenoid.yaml에서 조정한다.
        self.declare_parameter('gpio_pin', 23)
        self.declare_parameter('gpiochip', 4)
        self.declare_parameter('active_low', False)
        self.declare_parameter('release_timeout_s', 5.0)
        self.pin = self.get_parameter('gpio_pin').value
        self.chip_num = self.get_parameter('gpiochip').value
        self.active_low = self.get_parameter('active_low').value
        self.release_timeout_s = max(
            0.1, float(self.get_parameter('release_timeout_s').value))

        # 커플링 걸쇠는 그냥 밀어 넣으면 스프링 힘만으로
        # 잠긴다 — 이게 "잠금"이고 무통전 상태다(전원이 끊겨도 저절로
        # 안 풀리는 페일세이프). "해제"는 솔레노이드가 실제로 동작해
        # 걸쇠를 아래로 당겨 내리는 것이고, 이때 코일이 통전된다.
        # 2026-09-01 실제 잠금/해제 동작이 반대로 확인되어 GPIO 극성을
        # active-HIGH로 정정했다. 변수명은 논리 동작인 lock/release를 쓴다.
        self.lock_level = 1 if self.active_low else 0      # 무통전 = 잠금
        self.release_level = 0 if self.active_low else 1   # 통전 = 해제

        # Pi 5 기본값은 gpiochip4이며 다른 보드를 위해 대체 칩도 확인한다.
        self.h = None
        for chip in [self.chip_num, 0, 4]:
            try:
                self.h = lgpio.gpiochip_open(chip)
                self.chip_num = chip
                break
            except Exception as e:
                self.get_logger().warn(f'gpiochip{chip} open 실패: {e}')
        if self.h is None:
            self.get_logger().error('gpiochip 을 열 수 없습니다. 종료.')
            sys.exit(1)

        # 시작 시 출력은 반드시 안전한 잠금(무통전) 상태로 둔다. 예전
        # 코드는 여기를 "해제" 레벨로 초기화했는데, 그게 사실은 통전
        # 상태였던 게 나중에 밝혀졌다 — 미션이 아무 명령도 안 보내는
        # 동안(예: 벤치에 오래 켜둔 채 방치) 코일이 계속 뜨거워지는
        # 과열 문제(TODO.md 9번)의 원인이었다.
        try:
            lgpio.gpio_claim_output(self.h, self.pin, self.lock_level)
        except Exception as e:
            self.get_logger().error(f'GPIO{self.pin} claim 실패: {e}')
            sys.exit(1)

        self.locked = True

        self.sub = self.create_subscription(Bool, 'solenoid_cmd', self._on_cmd, 10)
        self.status_pub = self.create_publisher(Bool, 'solenoid_status', 10)
        self.status_timer = self.create_timer(1.0, self._publish_status)
        self.release_timer = self.create_timer(
            self.release_timeout_s, self._on_release_timeout)
        self.release_timer.cancel()

        self.get_logger().info(
            f'Solenoid Node started | gpiochip{self.chip_num} GPIO{self.pin} '
            f'{"active-LOW" if self.active_low else "active-HIGH"} | 초기 잠금(무통전)')

    def _apply(self, locked: bool):
        level = self.lock_level if locked else self.release_level
        try:
            lgpio.gpio_write(self.h, self.pin, level)
            self.locked = locked
            self.get_logger().info(
                f'[SOLENOID] {"잠금(무통전)" if locked else "해제(통전)"} '
                f'-> GPIO{self.pin}={level}')
        except Exception as e:
            self.get_logger().error(f'GPIO write 실패: {e}')

    def _on_cmd(self, msg: Bool):
        locked = bool(msg.data)
        self._apply(locked)
        if locked:
            self.release_timer.cancel()
        else:
            # 해제 상태로 방치되지 않도록 마지막 해제 명령부터 다시 센다.
            self.release_timer.reset()
            self.get_logger().info(
                f'[SOLENOID] {self.release_timeout_s:.1f}초 후 자동 잠금')

    def _on_release_timeout(self):
        # rclpy 타이머는 주기형이므로 먼저 취소해 일회성으로 사용한다.
        self.release_timer.cancel()
        if not self.locked:
            self.get_logger().info('[SOLENOID] 해제 시간 만료 — 자동 잠금')
            self._apply(True)

    def _publish_status(self):
        m = Bool()
        m.data = self.locked
        self.status_pub.publish(m)

    def shutdown(self):
        # 종료 시에도 안전한 잠금(무통전) 상태로 되돌린다 — 통전 상태로
        # 남겨두면(예전 방식) 프로세스가 죽어도 코일이 계속 뜨거워진다.
        if self.h is None:
            return
        try:
            lgpio.gpio_write(self.h, self.pin, self.lock_level)
            lgpio.gpiochip_close(self.h)
        except Exception:
            pass
        self.h = None


def main(args=None):
    rclpy.init(args=args)
    node = SolenoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
