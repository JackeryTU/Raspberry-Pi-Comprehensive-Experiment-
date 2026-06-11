"""
navigation/maneuvers.py
黄色和绿色魔方机动动作序列（v3）

策略：原地旋转式通过
  步骤序列：approach → stop → rotate1(±angle) → straight → rotate2(∓angle) → straight

  黄色（从左侧通过）：
    1. approach  — 距离PID接近到 stop_distance
    2. stop      — 停车 0.3s
    3. rotate1   — 原地左转（右进左退），角度 = yellow_rotate1_angle
    4. straight  — 直行通过 yellow_pass_dist cm（编码器判定）
    5. rotate2   — 原地右转回正（左进右退），角度 = yellow_rotate2_angle
    6. straight  — 短距恢复 recover_straight_dist cm（编码器判定）

  绿色（从右侧通过）：
    1. approach  — 距离PID接近到 stop_distance
    2. stop      — 停车 0.3s
    3. rotate1   — 原地右转（左进右退），角度 = green_rotate1_angle
    4. straight  — 直行通过 green_pass_dist cm（编码器判定）
    5. rotate2   — 原地左转回正（右进左退），角度 = green_rotate2_angle
    6. straight  — 短距恢复 recover_straight_dist cm（编码器判定）

红色魔方绕行逻辑不变，仍在 state_machine.py 中直接实现。
"""

from config import MANEUVER


class ManeuverStep:
    """
    单段机动定义（v3）。

    属性
    ----
    action         : 'approach' | 'stop' | 'rotate' | 'straight'
    speed          : 基准占空比 (0-100)
    turn_bias      : 旋转方向符号
                     rotate 时：+1 = 右转/CW（左轮前进，右轮反转）
                               -1 = 左转/CCW（右轮前进，左轮反转）
                     approach/straight 时：通常为 0（直行）
    target_angle   : rotate 时的目标角度（度），编码器脉冲之和判定
    target_distance: approach/straight 时的目标距离（cm）
                     approach：超声波判定（到魔方的距离）
                     straight：编码器判定（行驶距离）
                     0 = 不使用距离判定
    timeout        : 本段最大持续时间（秒），保底退出
    desc           : 调试描述
    """

    __slots__ = ('action', 'speed', 'turn_bias', 'target_angle',
                 'target_distance', 'timeout', 'desc')

    def __init__(self, action, speed=0, turn_bias=0, target_angle=0,
                 target_distance=0, timeout=1.0, desc=""):
        self.action         = action
        self.speed          = speed
        self.turn_bias      = turn_bias
        self.target_angle   = target_angle
        self.target_distance = target_distance
        self.timeout        = timeout
        self.desc           = desc

    def __repr__(self):
        parts = [f"action={self.action!r}"]
        if self.speed:
            parts.append(f"speed={self.speed}")
        if self.turn_bias:
            parts.append(f"turn_bias={self.turn_bias:+.0f}")
        if self.target_angle:
            parts.append(f"target_angle={self.target_angle}°")
        if self.target_distance:
            parts.append(f"target_distance={self.target_distance}cm")
        parts.append(f"timeout={self.timeout}s")
        return f"ManeuverStep({', '.join(parts)})"


def build_yellow():
    """
    黄色魔方：从左侧通过。
    approach → stop → rotate1(左转) → straight → rotate2(右转回正) → straight → straight(清离)
    """
    return [
        ManeuverStep(
            action='approach',
            speed=MANEUVER['approach_speed_yg'],
            turn_bias=0,
            target_distance=MANEUVER['stop_distance'],
            timeout=MANEUVER['approach_timeout'],
            desc="黄:距离PID接近"
        ),
        ManeuverStep(
            action='stop',
            timeout=MANEUVER['stop_time'],
            desc="黄:停车"
        ),
        ManeuverStep(
            action='rotate',
            speed=MANEUVER['yellow_rotate1_speed'],
            turn_bias=-1,                                   # -1 = 左转（右进左退）
            target_angle=MANEUVER['yellow_rotate1_angle'],
            timeout=MANEUVER['rotate_timeout'],
            desc="黄:原地左转"
        ),
        ManeuverStep(
            action='straight',
            speed=MANEUVER['yellow_pass_speed'],
            turn_bias=0,
            target_distance=MANEUVER['yellow_pass_dist'],
            timeout=MANEUVER['yellow_pass_time'],
            desc="黄:直行通过(左侧)"
        ),
        ManeuverStep(
            action='rotate',
            speed=MANEUVER['yellow_rotate2_speed'],
            turn_bias=+1,                                   # +1 = 右转（左进右退）
            target_angle=MANEUVER['yellow_rotate2_angle'],
            timeout=MANEUVER['rotate_timeout'],
            desc="黄:原地右转回正"
        ),
        ManeuverStep(
            action='straight',
            speed=MANEUVER['yellow_pass_speed'],
            turn_bias=0,
            target_distance=MANEUVER['recover_straight_dist'],
            timeout=1.0,
            desc="黄:直行恢复"
        ),
        ManeuverStep(
            action='straight',
            speed=MANEUVER['yellow_pass_speed'],
            turn_bias=0,
            target_distance=0,                                   # 0=纯时间判定
            timeout=MANEUVER['post_maneuver_straight_time'],
            desc="黄:通过后直行清离"
        ),
    ]


def build_green():
    """
    绿色魔方：从右侧通过。
    approach → stop → rotate1(右转) → straight → rotate2(左转回正) → straight → straight(清离)
    """
    return [
        ManeuverStep(
            action='approach',
            speed=MANEUVER['approach_speed_yg'],
            turn_bias=0,
            target_distance=MANEUVER['stop_distance'],
            timeout=MANEUVER['approach_timeout'],
            desc="绿:距离PID接近"
        ),
        ManeuverStep(
            action='stop',
            timeout=MANEUVER['stop_time'],
            desc="绿:停车"
        ),
        ManeuverStep(
            action='rotate',
            speed=MANEUVER['green_rotate1_speed'],
            turn_bias=+1,                                   # +1 = 右转（左进右退）
            target_angle=MANEUVER['green_rotate1_angle'],
            timeout=MANEUVER['rotate_timeout'],
            desc="绿:原地右转"
        ),
        ManeuverStep(
            action='straight',
            speed=MANEUVER['green_pass_speed'],
            turn_bias=0,
            target_distance=MANEUVER['green_pass_dist'],
            timeout=MANEUVER['green_pass_time'],
            desc="绿:直行通过(右侧)"
        ),
        ManeuverStep(
            action='rotate',
            speed=MANEUVER['green_rotate2_speed'],
            turn_bias=-1,                                   # -1 = 左转（右进左退）
            target_angle=MANEUVER['green_rotate2_angle'],
            timeout=MANEUVER['rotate_timeout'],
            desc="绿:原地左转回正"
        ),
        ManeuverStep(
            action='straight',
            speed=MANEUVER['green_pass_speed'],
            turn_bias=0,
            target_distance=MANEUVER['recover_straight_dist'],
            timeout=1.0,
            desc="绿:直行恢复"
        ),
        ManeuverStep(
            action='straight',
            speed=MANEUVER['green_pass_speed'],
            turn_bias=0,
            target_distance=0,                                   # 0=纯时间判定
            timeout=MANEUVER['post_maneuver_straight_time'],
            desc="绿:通过后直行清离"
        ),
    ]


# 颜色 → 动作序列映射表
# 注意：红色不在此表中，由 state_machine.py 直接处理
MANEUVER_TABLE = {
    'yellow': build_yellow(),
    'green':  build_green(),
}
