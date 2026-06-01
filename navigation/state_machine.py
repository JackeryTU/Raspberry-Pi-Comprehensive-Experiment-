"""
navigation/state_machine.py
魔方绕桩主状态机 v2

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
状态流转总览
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
黄/绿魔方（定时+超声退出）：
  SEARCH → ALIGN → PASS → RECOVER → SEARCH / FINISH

红色魔方（"切向入轨"策略）：
  SEARCH → ALIGN_RED → APPROACH → TANGENT → ORBIT → RECOVER_RED → SEARCH / FINISH

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
红色绕行策略详解（切向入轨）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. ALIGN_RED  ── 摄像头比例对准
   用P控制器把红色魔方对准到画面中心（cx误差→转向偏置）。
   当超声波距离 < orbit_radius + approach_margin 时退出，进入 APPROACH。

2. APPROACH   ── 直线接近到入轨距离
   车头已对准魔方中心，直线行驶直到 dist ≤ orbit_radius（cm）。
   此时小车距魔方中心正好是绕行半径。

3. TANGENT    ── 原地右转90°（切向对齐）
   以较大的差速右转（两轮均正转，左轮快）直到编码器累计角度≥90°。
   转完后：魔方在车身右侧，车头朝向绕圈切线方向（逆时针轨道出发方向）。

4. ORBIT      ── 固定差速弧线绕行360°
   用固定 (orbit_speed, orbit_turn_bias) 右转差速驱动。
   退出条件（主）：编码器累计右转角度 ≥ 360°
   退出条件（辅）：摄像头再次检测到红色魔方（确认回到起始面）
   退出条件（保底）：超时

5. RECOVER_RED ── 简单回正，然后进入 SEARCH 或 FINISH

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
黄/绿步骤设计（修复原版 _advance() bug）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
原版 _advance() 的 bug：从 ALIGN 推进时会在单 tick 内把 pass 和 recover 步骤
全部跳过，导致 PASS 和 RECOVER 状态始终无步骤可执行。

修复方案：
  _steps = [align_step(idx=0), pass_step(idx=1), recover_step(idx=2)]
  • ALIGN   state 使用 steps[0]，完成后 step_idx→1，_enter(PASS)
  • PASS    state 使用 steps[1]，完成后 step_idx→2，_enter(RECOVER)
  • RECOVER state 使用 steps[2]，完成后 cubes_passed++，_enter(SEARCH/FINISH)
  每次 _complete_step() 只递增一次，按新 step 的 phase 切换到对应状态。
"""

import time
import sys
import os
from enum import Enum, auto
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import MOTOR, ULTRASONIC, MANEUVER
from sensors.camera import Camera, identify_nearest_cube, detect_cube
from sensors.ks103 import KS103
from sensors.encoder import get_encoder_counts
from control.motor import PIDSpeedLoop, stop as motor_stop


# ──────────────────────────────────────────────────────
class State(Enum):
    SEARCH      = auto()   # 直线搜索，摄像头扫色
    ALIGN       = auto()   # 黄/绿：切向转向（ALIGN步骤）
    PASS        = auto()   # 黄/绿：侧身通过（PASS步骤）
    RECOVER     = auto()   # 黄/绿：回正（RECOVER步骤）
    ALIGN_RED   = auto()   # 红色：摄像头比例对准 + 接近触发
    APPROACH    = auto()   # 红色：直线接近到入轨半径
    TANGENT     = auto()   # 红色：原地右转90°切向对齐
    ORBIT       = auto()   # 红色：360°弧线绕行
    RECOVER_RED = auto()   # 红色：回正后进入SEARCH
    FINISH      = auto()   # 任务完成


# 步骤 phase → 对应状态（供 _complete_step 使用）
_PHASE_TO_STATE = {
    'align':   State.ALIGN,
    'pass':    State.PASS,
    'recover': State.RECOVER,
}
# ──────────────────────────────────────────────────────


class CubeStateMachine:
    """魔方绕桩状态机 v2"""

    def __init__(self, camera: Camera, sonar: KS103):
        self.cam   = camera
        self.sonar = sonar
        self.loop  = PIDSpeedLoop()

        self.state        = State.SEARCH
        self.cubes_passed = 0
        self.target_color: Optional[str] = None

        # 黄/绿机动步骤序列（列表中顺序对应 ALIGN/PASS/RECOVER）
        self._steps    = []
        self._step_idx = 0

        # 当前状态进入时的时间与编码器基准
        self._t0:     float = 0.0
        self._enc0_r: int   = 0
        self._enc0_l: int   = 0

        # 超声波历史（PASS 退出判定：距离先减后增）
        self._dist_hist = []

        # 编码器转角标定系数（必须用 cal_angle.py 实车标定！）
        self._K: float = MANEUVER.get('deg_per_pulse_diff', 0.2)

    # ════════════════════════════════════════════
    #  内部工具方法
    # ════════════════════════════════════════════

    def _enter(self, new_state: State):
        """进入新状态，重置计时器与编码器基准。"""
        self.state    = new_state
        self._t0      = time.monotonic()
        self._dist_hist = []
        r, l = get_encoder_counts()
        self._enc0_r = r
        self._enc0_l = l
        print(f"[FSM] >>> {new_state.name}")

    def _elapsed(self) -> float:
        return time.monotonic() - self._t0

    def _enc_delta(self):
        """返回 (右轮脉冲差, 左轮脉冲差) 自进入当前状态以来的累计量。"""
        r, l = get_encoder_counts()
        return r - self._enc0_r, l - self._enc0_l

    def _rot_angle(self) -> float:
        """
        估算车身累计右转角度（度）。
        差速右转时左轮快(dl>dr) → 返回正值（右转）。
        差速左转时右轮快(dr>dl) → 返回负值（左转）。
        """
        dr, dl = self._enc_delta()
        return (dl - dr) * self._K

    def _current_step(self):
        """返回当前机动步骤，超出索引时返回 None。"""
        if self._step_idx < len(self._steps):
            return self._steps[self._step_idx]
        return None

    def _load_maneuver(self, color: str):
        """从 maneuvers.py 按颜色加载步骤序列，并重置索引。"""
        from navigation.maneuvers import MANEUVER_TABLE
        self._steps    = MANEUVER_TABLE.get(color, [])
        self._step_idx = 0
        print(f"[FSM] 加载 {color} 机动序列：{len(self._steps)} 步")

    def _complete_step(self):
        """
        黄/绿专用：完成当前步骤，推进到下一步并切换状态。
        修复了原版 _advance() bug：每次只递增一步，按新步骤 phase 切换状态。
        """
        self._step_idx += 1
        step = self._current_step()
        if step is None:
            # 序列耗尽 → 计入已过魔方，继续下一轮搜索或结束
            self.cubes_passed += 1
            print(f"[FSM] 已通过 {self.cubes_passed}/3 个魔方")
            self._enter(State.FINISH if self.cubes_passed >= 3 else State.SEARCH)
        else:
            # 按下一步的 phase 切换状态
            next_state = _PHASE_TO_STATE.get(step.phase, State.RECOVER)
            self._enter(next_state)

    def _check_pass_exit(self, dist: Optional[float]) -> bool:
        """
        检测黄/绿 PASS 阶段的"先减后增"退出特征。
        曾经靠近过魔方(dist < action_dist*1.2) 且 现在已远离(dist > clear_dist)。
        """
        if dist is None:
            return False
        self._dist_hist.append(dist)
        if len(self._dist_hist) > 20:
            self._dist_hist.pop(0)
        ever_close = any(d < ULTRASONIC['action_dist'] * 1.2
                         for d in self._dist_hist[:-3])
        now_clear  = dist > ULTRASONIC['clear_dist']
        return ever_close and now_clear

    # ════════════════════════════════════════════
    #  主循环 tick()
    # ════════════════════════════════════════════

    def tick(self) -> bool:
        """
        状态机单步推进，由主循环以 10~20 Hz 频率调用。
        返回 True：继续运行；False：任务完成（FINISH）或故障。
        """
        frame = self.cam.get_frame()
        dist  = self.sonar.get_distance()  # cm，可能为 None
        step  = self._current_step()

        # ─── 公共 DEBUG 日志（降低刷屏频率可外部控制）───
        dist_s = f"{dist:.1f}" if dist is not None else "None"
        print(f"[FSM] {self.state.name:12s}  dist={dist_s:>6} cm  "
              f"step_idx={self._step_idx}  t={self._elapsed():.1f}s")

        # ════ SEARCH ════
        if self.state == State.SEARCH:
            self.loop.set_target(MOTOR['base_speed'], 0.0)

            if frame is not None and dist is not None and dist < ULTRASONIC['detect_dist']:
                color, cx, area = identify_nearest_cube(frame)
                if color:
                    print(f"[FSM] 检测到 {color} 魔方  cx={cx}  area={area}  dist={dist:.1f}cm")
                    self.target_color = color
                    if color == 'red':
                        self._enter(State.ALIGN_RED)
                    else:
                        self._load_maneuver(color)
                        self._enter(State.ALIGN)
            return True

        # ════ ALIGN（黄/绿：切向转向） ════
        elif self.state == State.ALIGN:
            if step is None or step.phase != 'align':
                # 步骤不匹配（理论上不应出现），安全跳过
                print("[FSM] ALIGN: 无 align 步骤，强制推进")
                self._complete_step()
                return True

            self.loop.set_target(step.speed, step.turn_bias)

            # 退出：超时 或 超声波进入动作距离
            if self._elapsed() > step.timeout:
                print(f"[FSM] ALIGN 超时 ({step.desc})")
                self._complete_step()
            elif dist is not None and dist < ULTRASONIC['action_dist']:
                print(f"[FSM] ALIGN 距离触发 ({step.desc})")
                self._complete_step()
            return True

        # ════ PASS（黄/绿：侧身通过） ════
        elif self.state == State.PASS:
            if step is None or step.phase != 'pass':
                print("[FSM] PASS: 无 pass 步骤，强制推进")
                self._complete_step()
                return True

            self.loop.set_target(step.speed, step.turn_bias)

            # 退出：超时 或 超声波"先减后增"脱离特征
            if self._elapsed() > step.timeout:
                print(f"[FSM] PASS 超时 ({step.desc})")
                self._complete_step()
            elif self._check_pass_exit(dist):
                print(f"[FSM] PASS 先减后增退出 ({step.desc})")
                self._complete_step()
            return True

        # ════ RECOVER（黄/绿：回正） ════
        elif self.state == State.RECOVER:
            if step is None or step.phase != 'recover':
                print("[FSM] RECOVER: 无 recover 步骤，强制计数推进")
                self.cubes_passed += 1
                print(f"[FSM] 已通过 {self.cubes_passed}/3 个魔方")
                self._enter(State.FINISH if self.cubes_passed >= 3 else State.SEARCH)
                return True

            self.loop.set_target(step.speed, step.turn_bias)

            if self._elapsed() > step.timeout:
                print(f"[FSM] RECOVER 完成 ({step.desc})")
                self._complete_step()
            return True

        # ════════════════════════════════════════
        #  红色魔方专用状态
        # ════════════════════════════════════════

        # ════ ALIGN_RED（摄像头比例对准） ════
        elif self.state == State.ALIGN_RED:
            # 用摄像头P控制器把红色魔方对准到画面中心
            if frame is not None:
                cx, _ = detect_cube(frame, 'red')
                h, w  = frame.shape[:2]
                if cx is not None:
                    # 归一化误差 [-1, 1]，正 = 魔方偏右 → 需右转
                    err       = (cx - w / 2.0) / (w / 2.0)
                    turn_bias = err * MANEUVER['align_centering_kp']
                    self.loop.set_target(MANEUVER['align_red_speed'], turn_bias)
                else:
                    # 丢失目标时低速直行保持探索
                    self.loop.set_target(MOTOR['base_speed'] * 0.7, 0.0)
            else:
                self.loop.set_target(MOTOR['base_speed'] * 0.7, 0.0)

            # 进入接近阶段：距离已近到 orbit_radius + margin + 10cm
            threshold = MANEUVER['orbit_radius'] + MANEUVER['approach_margin'] + 10
            if dist is not None and dist < threshold:
                print(f"[FSM] ALIGN_RED: 距离 {dist:.1f}cm < {threshold}cm，进入 APPROACH")
                self._enter(State.APPROACH)
            elif self._elapsed() > MANEUVER['align_red_timeout']:
                print("[FSM] ALIGN_RED 超时，重新 SEARCH")
                self._enter(State.SEARCH)
            return True

        # ════ APPROACH（直线接近到入轨距离） ════
        elif self.state == State.APPROACH:
            self.loop.set_target(MANEUVER['approach_speed'], 0.0)

            target_dist = MANEUVER['orbit_radius']
            if dist is not None and dist <= target_dist:
                print(f"[FSM] APPROACH: 到达入轨距离 {dist:.1f}cm，进入 TANGENT")
                self._enter(State.TANGENT)
            elif self._elapsed() > MANEUVER['approach_timeout']:
                print("[FSM] APPROACH 超时，强制进入 TANGENT")
                self._enter(State.TANGENT)
            return True

        # ════ TANGENT（原地右转90°，切向对齐） ════
        elif self.state == State.TANGENT:
            # 差速右转：左轮快，右轮慢，两轮均正转
            self.loop.set_target(MANEUVER['tangent_speed'],
                                 MANEUVER['tangent_turn_bias'])

            angle = self._rot_angle()   # 正值 = 右转
            print(f"[FSM] TANGENT  angle={angle:+.1f}°  target=+{MANEUVER['tangent_target_deg']}°")

            if angle >= MANEUVER['tangent_target_deg']:
                print("[FSM] TANGENT 完成：已切向对齐，进入 ORBIT")
                self._enter(State.ORBIT)
            elif self._elapsed() > MANEUVER['tangent_timeout']:
                print("[FSM] TANGENT 超时，强制进入 ORBIT")
                self._enter(State.ORBIT)
            return True

        # ════ ORBIT（固定差速弧线绕行360°） ════
        elif self.state == State.ORBIT:
            self.loop.set_target(MANEUVER['orbit_speed'],
                                 MANEUVER['orbit_turn_bias'])

            angle = self._rot_angle()   # 右转正值
            print(f"[FSM] ORBIT  angle={angle:+.1f}°/360°  t={self._elapsed():.1f}s")

            # 主退出：编码器累计右转 ≥ 360°
            if angle >= 360.0:
                # 可选辅助验证：摄像头再次看到魔方（确认已回到起始面侧）
                confirmed = False
                if frame is not None:
                    cx, _ = detect_cube(frame, 'red')
                    if cx is not None:
                        confirmed = True
                        print("[FSM] ORBIT 完成（编码器+摄像头双重确认）")
                    else:
                        print("[FSM] ORBIT 完成（编码器确认，摄像头未见魔方）")
                else:
                    print("[FSM] ORBIT 完成（编码器确认）")
                self._enter(State.RECOVER_RED)

            # 保底退出：超时
            elif self._elapsed() > MANEUVER['orbit_timeout']:
                print("[FSM] ORBIT 超时，强制退出")
                self._enter(State.RECOVER_RED)
            return True

        # ════ RECOVER_RED（红色专用回正） ════
        elif self.state == State.RECOVER_RED:
            # 绕行完成后，车身姿态应已近似回到出发方向
            # 做一段轻微回正转向（与最后一小段轨迹修正）
            self.loop.set_target(MANEUVER['recover_speed'],
                                 -MANEUVER['recover_turn'])  # 左转回正

            if self._elapsed() > MANEUVER['recover_time']:
                self.cubes_passed += 1
                print(f"[FSM] 红色绕行完成，已通过 {self.cubes_passed}/3 个魔方")
                self._enter(State.FINISH if self.cubes_passed >= 3 else State.SEARCH)
            return True

        # ════ FINISH ════
        elif self.state == State.FINISH:
            self.loop.set_target(0, 0)
            motor_stop()
            print("[FSM] ★★ 任务完成！到达终点 ★★")
            return False

        # 未知状态保护
        print(f"[FSM] 未知状态 {self.state}，强制 SEARCH")
        self._enter(State.SEARCH)
        return True

    # ════════════════════════════════════════════
    #  生命周期
    # ════════════════════════════════════════════

    def start(self):
        """启动 PID 闭环，进入 SEARCH 状态。"""
        self.loop.start()
        self._enter(State.SEARCH)

    def stop(self):
        """安全停止。"""
        self.loop.stop()
