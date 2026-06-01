"""
navigation/maneuvers.py
黄色和绿色魔方机动动作序列

说明
----
• 黄/绿魔方通过动作被拆分为 3 个 ManeuverStep，按 step_idx 顺序被状态机消费：
    index 0  phase='align'   — 切向转向阶段（ALIGN 状态使用）
    index 1  phase='pass'    — 侧身通过阶段（PASS  状态使用）
    index 2  phase='recover' — 回正阶段（RECOVER 状态使用）

• 红色魔方绕行（360°逆时针）逻辑复杂，全部在 state_machine.py 中直接实现，
  此处不再提供红色动作序列。

• 每个状态机只从 MANEUVER_TABLE 按颜色取出对应列表后，在对应的
  ALIGN / PASS / RECOVER 状态中依次读取 steps[step_idx]，执行完毕后
  推进 step_idx 并切换状态。详见 state_machine.py 的 _complete_step()。
"""

from config import MANEUVER


class ManeuverStep:
    """
    单段机动定义。

    属性
    ----
    phase     : 对应的状态机阶段 ('align' | 'pass' | 'recover')
    speed     : 目标速度占空比（直接用于 PIDSpeedLoop.set_target）
    turn_bias : 转向偏置 (-1~1)，已含方向符号
    timeout   : 本段最大持续时间（秒）
    desc      : 调试描述
    """

    __slots__ = ('phase', 'speed', 'turn_bias', 'timeout', 'desc')

    def __init__(self, phase, speed, turn_bias, timeout, desc=""):
        self.phase     = phase
        self.speed     = speed
        self.turn_bias = turn_bias
        self.timeout   = timeout
        self.desc      = desc

    def __repr__(self):
        return (f"ManeuverStep(phase={self.phase!r}, speed={self.speed}, "
                f"turn_bias={self.turn_bias:+.2f}, timeout={self.timeout}s)")


def build_yellow():
    """
    黄色魔方：从左侧通过。
    左转切向 → 左侧侧身直行 → 右转回正。
    """
    return [
        ManeuverStep(
            phase='align',
            speed=MANEUVER['align_speed'],
            turn_bias=-MANEUVER['align_turn'],   # 负 = 左转
            timeout=MANEUVER['align_time'],
            desc="黄:左转切向"
        ),
        ManeuverStep(
            phase='pass',
            speed=MANEUVER['pass_speed'],
            turn_bias=-MANEUVER['pass_turn'],    # 轻微左转保持间距
            timeout=MANEUVER['pass_time'],
            desc="黄:左侧通过"
        ),
        ManeuverStep(
            phase='recover',
            speed=MANEUVER['recover_speed'],
            turn_bias=+MANEUVER['recover_turn'], # 右转回正
            timeout=MANEUVER['recover_time'],
            desc="黄:右转回正"
        ),
    ]


def build_green():
    """
    绿色魔方：从右侧通过。
    右转切向 → 右侧侧身直行 → 左转回正。
    """
    return [
        ManeuverStep(
            phase='align',
            speed=MANEUVER['align_speed'],
            turn_bias=+MANEUVER['align_turn'],   # 正 = 右转
            timeout=MANEUVER['align_time'],
            desc="绿:右转切向"
        ),
        ManeuverStep(
            phase='pass',
            speed=MANEUVER['pass_speed'],
            turn_bias=+MANEUVER['pass_turn'],    # 轻微右转保持间距
            timeout=MANEUVER['pass_time'],
            desc="绿:右侧通过"
        ),
        ManeuverStep(
            phase='recover',
            speed=MANEUVER['recover_speed'],
            turn_bias=-MANEUVER['recover_turn'], # 左转回正
            timeout=MANEUVER['recover_time'],
            desc="绿:左转回正"
        ),
    ]


# 颜色 → 动作序列映射表
# 注意：红色不在此表中，由 state_machine.py 直接处理
MANEUVER_TABLE = {
    'yellow': build_yellow(),
    'green':  build_green(),
}
