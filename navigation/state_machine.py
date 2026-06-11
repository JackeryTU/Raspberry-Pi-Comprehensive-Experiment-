"""
navigation/state_machine.py
魔方绕桩主状态机 v3.9

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
状态流转总览
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
黄/绿魔方（原地旋转式通过，v3）：
  SEARCH（含扫描）→ MANEUVER → POST_WAIT → SCAN_NEXT → ALIGN_CUBE → SEARCH / FINISH
  MANEUVER 内部: approach → stop → rotate1 → straight → rotate2 → straight

红色魔方（切向入轨策略）：
  SEARCH（含扫描）→ ALIGN_RED → APPROACH → TANGENT → ORBIT → RECOVER_RED
    → POST_WAIT → SCAN_NEXT → ALIGN_CUBE → SEARCH / FINISH

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
完成魔方后的扫描与对准（v3.1 新增）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
每通过一个魔方后，不直接进入 SEARCH，而是：
  1. POST_WAIT  — 停车等待 post_maneuver_wait 秒，消除惯性
  2. SCAN_NEXT  — 原地旋转扫描（左→右），纯摄像头检测下一个魔方
  3. ALIGN_CUBE — PID 摄像头对准魔方中心 + 前进接近到 align_cube_stop_dist
  4. 进入 SEARCH，由 SEARCH 快速检测并路由到对应机动

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
SEARCH 三阶段扫描（v3）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
解决魔方非直线排列时的遗漏问题：
  1. straight    — 直线前进，超声波+摄像头检测，持续 search_straight_time 秒
  2. scan_left   — 原地慢速左转 scan_angle°，纯摄像头检测
  3. scan_right  — 原地慢速右转 scan_angle×2°，纯摄像头检测
  任一阶段摄像头发现魔方 → 立即锁定，进入对应机动

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
黄/绿机动策略详解（v3）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. APPROACH  — 直线接近到 action_dist（超声波判定）
2. STOP      — 短暂停车消除惯性
3. ROTATE1   — 原地旋转（接近转向），角度/速度按颜色独立配置
4. STRAIGHT  — PID 闭环直行通过
5. ROTATE2   — 原地旋转（回正），角度/速度按颜色独立配置
6. STRAIGHT  — 短距直行恢复

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
红色绕行策略详解（切向入轨）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. ALIGN_RED  ── 摄像头比例对准
2. APPROACH   ── 直线接近到入轨距离（红色专用，不同于黄绿的 approach）
3. TANGENT    ── 原地右转90°切向对齐
4. ORBIT      ── 固定差速弧线逆时针绕行
5. RECOVER_RED ── 简单回正，进入 POST_WAIT
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
    SEARCH      = auto()   # 三阶段扫描搜索（straight → scan_left → scan_right）
    MANEUVER    = auto()   # 黄/绿：原地旋转式通过（v3）
    ALIGN_RED   = auto()   # 红色：摄像头比例对准 + 接近触发
    APPROACH    = auto()   # 红色：直线接近到入轨半径
    TANGENT     = auto()   # 红色：原地右转90°切向对齐
    ORBIT       = auto()   # 红色：弧线绕行
    RECOVER_RED = auto()   # 红色：回正后进入POST_WAIT
    POST_WAIT   = auto()   # v3.1：完成魔方后等待，消除惯性
    SCAN_NEXT   = auto()   # v3.1：原地旋转扫描下一个魔方
    ALIGN_CUBE  = auto()   # v3.1：PID对准魔方中心，接近后进入SEARCH
    FINISH      = auto()   # 任务完成
# ──────────────────────────────────────────────────────


class CubeStateMachine:
    """魔方绕桩状态机 v3.9"""

    def __init__(self, camera: Camera, sonar: KS103):
        self.cam   = camera
        self.sonar = sonar
        self.loop  = PIDSpeedLoop()

        self.state        = State.SEARCH
        self.cubes_passed = 0
        self.target_color: Optional[str] = None

        # 机动步骤序列（由 maneuvers.py 按颜色加载）
        self._steps    = []
        self._step_idx = 0

        # SEARCH 扫描阶段
        # 'straight' | 'scan_left' | 'scan_right'
        self._search_phase = 'straight'

        # SCAN_NEXT 扫描阶段（v3.1）
        # 'left' | 'right'
        self._scan_next_phase = 'left'

        # SCAN_NEXT 中检测到的下一个魔方颜色（v3.1）
        self._next_color: Optional[str] = None

        # 刚通过魔方的颜色，SCAN_NEXT 中忽略此颜色防止重复检测（v3.1）
        self._last_passed_color: Optional[str] = None

        # APPROACH settle 阶段总计时起点（超时保护）
        self._settle_total_start: Optional[float] = None

        # ALIGN_CUBE PD 控制的上一次偏差（v3.1）
        self._align_prev_err: float = 0.0

        # TANGENT 两阶段：'stop' → 'rotate'（v3.1 改为原地旋转）
        self._tangent_phase = 'stop'

        # ALIGN_CUBE 两阶段：'rotate'（原地PID对准）→ 'forward'（直行接近）
        self._align_phase = 'rotate'

        # 当前状态/步骤进入时的时间与编码器基准
        self._t0:     float = 0.0
        self._enc0_r: int   = 0
        self._enc0_l: int   = 0

        # 编码器转角标定系数
        self._K: float = MANEUVER.get('deg_per_pulse_diff', 0.2)

        # 原地旋转标定系数（度/脉冲之和）
        self._K_rotate: float = MANEUVER.get('deg_per_pulse_rotate', 0.14)

        # 编码器直行距离标定系数
        self._pulses_per_cm: float = MANEUVER.get('pulses_per_cm', 11.7)

        # 距离 PID settle 计时器（approach 到位后稳定计时）
        self._settle_start: Optional[float] = None

    # ════════════════════════════════════════════
    #  内部工具方法
    # ════════════════════════════════════════════

    def _enter(self, new_state: State):
        """进入新状态，重置计时器与编码器基准。"""
        self.state    = new_state
        self._t0      = time.monotonic()
        r, l = get_encoder_counts()
        self._enc0_r = r
        self._enc0_l = l
        if new_state == State.SEARCH:
            self._search_phase = 'straight'
        if new_state == State.SCAN_NEXT:
            self._scan_next_phase = 'left'
        if new_state == State.POST_WAIT:
            self.loop.set_target(0, 0)  # 立即停车
        if new_state == State.ALIGN_CUBE:
            self._align_prev_err = 0.0
            self._align_phase = 'rotate'
        if new_state == State.TANGENT:
            self._tangent_phase = 'stop'
            self.loop.set_target(0, 0)  # 先停车
        if new_state == State.APPROACH:
            # 红色 APPROACH 状态复用了 _settle_start / _settle_total_start，
            # 必须在进入时清零，防止与黄/绿 MANEUVER 的残留值冲突。
            self._settle_start = None
            self._settle_total_start = None
        print(f"[FSM] >>> {new_state.name}")

    def _start_search_phase(self, phase: str):
        """切换 SEARCH 子阶段，重置计时与编码器基准，并先停车。"""
        self.loop.set_target(0, 0)  # ★ 先停车，防止上一阶段电机指令残留
        self._search_phase = phase
        self._t0 = time.monotonic()
        r, l = get_encoder_counts()
        self._enc0_r = r
        self._enc0_l = l
        print(f"[FSM] SEARCH.{phase}")

    def _elapsed(self) -> float:
        return time.monotonic() - self._t0

    def _enc_delta(self):
        """返回 (右轮脉冲差, 左轮脉冲差) 自当前基准以来的累计量。"""
        r, l = get_encoder_counts()
        return r - self._enc0_r, l - self._enc0_l

    def _rot_angle(self) -> float:
        """
        估算车身累计右转角度（度）。
        差速右转时左轮快(dl>dr) → 返回正值（右转）。
        """
        dr, dl = self._enc_delta()
        return (dr - dl) * self._K

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
        for s in self._steps:
            print(f"       {s.desc}")

    def _advance_step(self):
        """
        推进到下一步骤。
        若序列耗尽 → cubes_passed++ → FINISH 或 POST_WAIT（v3.1）。
        否则重置每步的时间/编码器基准（复用 _t0 / _enc0）。
        """
        self._settle_start = None       # 清除距离 PID settle 状态
        self._settle_total_start = None # 清除 settle 总超时
        self._step_idx += 1
        step = self._current_step()
        if step is None:
            self.cubes_passed += 1
            print(f"[FSM] 已通过 {self.cubes_passed}/3 个魔方")
            if self.cubes_passed >= 3:
                self._enter(State.FINISH)
            else:
                # v3.1：记录刚通过的颜色，SCAN_NEXT 中忽略它
                self._last_passed_color = self.target_color
                self._next_color = None
                self._enter(State.POST_WAIT)
        else:
            self._t0 = time.monotonic()
            r, l = get_encoder_counts()
            self._enc0_r = r
            self._enc0_l = l
            print(f"[FSM] → {step.desc}")

    # ════════════════════════════════════════════
    #  摄像头检测辅助
    # ════════════════════════════════════════════

    def _try_detect_cube(self, frame, require_distance: bool, dist,
                         ignore_color: Optional[str] = None) -> Optional[str]:
        """
        尝试检测魔方。
        require_distance=True  ：需要超声波 dist < detect_dist（直行阶段用）
        require_distance=False ：纯摄像头检测（扫描阶段用）
        ignore_color           ：忽略此颜色（防止 SCAN_NEXT 重复检测刚通过的魔方）
        返回颜色名，未检测到返回 None。
        """
        if frame is None:
            return None
        if require_distance and (dist is None or dist >= ULTRASONIC['detect_dist']):
            return None
        color, cx, area = identify_nearest_cube(frame)
        if color:
            if ignore_color and color == ignore_color:
                # 忽略刚通过的魔方颜色，避免重复检测
                print(f"[FSM] 检测到 {color} 魔方（已忽略，与刚通过的魔方同色）"
                      f"  cx={cx}  area={area}")
                return None
            d_s = f"{dist:.1f}cm" if dist is not None else "---"
            print(f"[FSM] 检测到 {color} 魔方  cx={cx}  area={area}  dist={d_s}")
        return color

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

        # ─── 公共 DEBUG 日志 ───
        dist_s = f"{dist:.1f}" if dist is not None else "None"
        step_desc = step.desc if step else "None"
        phase_s = f" [{self._search_phase}]" if self.state == State.SEARCH else ""
        print(f"[FSM] {self.state.name:12s}{phase_s}  dist={dist_s:>6} cm  "
              f"step_idx={self._step_idx}  t={self._elapsed():.1f}s  "
              f"step={step_desc}")

        # ════════════════════════════════════════════
        #  SEARCH — 三阶段扫描（v3 重写）
        #  1. straight   → 直行，超声波+摄像头
        #  2. scan_left  → 原地左转，纯摄像头
        #  3. scan_right → 原地右转，纯摄像头
        # ════════════════════════════════════════════
        if self.state == State.SEARCH:

            # ── 阶段 1：直线搜索 ──
            if self._search_phase == 'straight':
                self.loop.set_target(MOTOR['base_speed'], 0.0)

                color = self._try_detect_cube(frame, require_distance=True, dist=dist)
                if color:
                    self.target_color = color
                    if color == 'red':
                        self._enter(State.ALIGN_RED)
                    else:
                        self._load_maneuver(color)
                        self._enter(State.MANEUVER)
                    return True

                if self._elapsed() > MANEUVER['search_straight_time']:
                    self._start_search_phase('scan_left')
                return True

            # ── 阶段 2：原地左转扫描（纯摄像头）──
            elif self._search_phase == 'scan_left':
                # 左转：右轮前进(+)，左轮反转(-)
                self.loop.set_raw(-MANEUVER['scan_speed'],
                                   MANEUVER['scan_speed'])

                color = self._try_detect_cube(frame, require_distance=False, dist=dist)
                if color:
                    self.loop.set_target(0, 0)  # 先停
                    self.target_color = color
                    if color == 'red':
                        self._enter(State.ALIGN_RED)
                    else:
                        self._load_maneuver(color)
                        self._enter(State.MANEUVER)
                    return True

                # 角度判定
                dr, dl = self._enc_delta()
                angle = (abs(dr) + abs(dl)) * self._K_rotate
                if angle >= MANEUVER['scan_angle']:
                    self._start_search_phase('scan_right')
                elif self._elapsed() > MANEUVER['scan_phase_timeout']:
                    # 超时保底：电机可能堵转，强制进入下一阶段
                    print(f"[FSM] SEARCH.scan_left 超时 "
                          f"({MANEUVER['scan_phase_timeout']}s) angle={angle:.1f}°，"
                          f"强制进入 scan_right")
                    self._start_search_phase('scan_right')
                return True

            # ── 阶段 3：原地右转扫描（纯摄像头，双倍角度以覆盖右侧+回中）──
            elif self._search_phase == 'scan_right':
                # 右转：左轮前进(+)，右轮反转(-)
                self.loop.set_raw(MANEUVER['scan_speed'],
                                  -MANEUVER['scan_speed'])

                color = self._try_detect_cube(frame, require_distance=False, dist=dist)
                if color:
                    self.loop.set_target(0, 0)
                    self.target_color = color
                    if color == 'red':
                        self._enter(State.ALIGN_RED)
                    else:
                        self._load_maneuver(color)
                        self._enter(State.MANEUVER)
                    return True

                # 角度判定：双倍扫描覆盖右侧
                dr, dl = self._enc_delta()
                angle = (abs(dr) + abs(dl)) * self._K_rotate
                if angle >= MANEUVER['scan_angle'] * 2:
                    # 扫描完毕未发现，回到直行
                    self._start_search_phase('straight')
                elif self._elapsed() > MANEUVER['scan_phase_timeout'] * 2:
                    # 超时保底：右扫角度是左扫两倍，超时也相应翻倍
                    print(f"[FSM] SEARCH.scan_right 超时 "
                          f"({MANEUVER['scan_phase_timeout'] * 2:.0f}s) angle={angle:.1f}°，"
                          f"强制回 straight")
                    self._start_search_phase('straight')
                return True

            # 未知阶段保护
            self._start_search_phase('straight')
            return True

        # ════════════════════════════════════════════
        #  MANEUVER — 黄/绿机动执行（v3）
        #  步骤: approach → stop → rotate1 → straight → rotate2 → straight
        # ════════════════════════════════════════════
        elif self.state == State.MANEUVER:
            if step is None:
                print("[FSM] MANEUVER: 无步骤，强制推进")
                self.cubes_passed += 1
                if self.cubes_passed >= 3:
                    self._enter(State.FINISH)
                else:
                    self._last_passed_color = self.target_color
                    self._next_color = None
                    self._enter(State.POST_WAIT)
                return True

            action = step.action

            # ── APPROACH：距离 PID 控制，精确停在 stop_distance ──
            #   error>0（太远）→ set_target(+speed) 前进 PID 闭环
            #   error<0（冲过头）→ set_target(-speed) 倒车直通（编码器不区分方向）
            #   首次到位后开始 settle，持续振荡超过 settle_timeout 则强制推进
            if action == 'approach':
                if dist is not None and step.target_distance > 0:
                    error = dist - step.target_distance   # >0 太远需前进，<0 太近需倒车
                    settling = self._settle_start is not None

                    # ── 首次进入时启动总超时计时（仅一次）──
                    if self._settle_total_start is None:
                        self._settle_total_start = time.monotonic()

                    # ── settle 总超时保护：从进入 approach 步骤开始计时 ──
                    if time.monotonic() - self._settle_total_start > MANEUVER['approach_settle_timeout']:
                        print(f"[FSM] APPROACH 总超时 "
                              f"({MANEUVER['approach_settle_timeout']}s)，强制推进")
                        self._advance_step()
                        return True

                    if settling:
                        # ── 已在 settle 状态 ──
                        # 滞回：小噪声不重置 settle，只有大幅偏离才重新追
                        if abs(error) > MANEUVER['approach_stop_tol'] * 2:
                            # 显著偏离，退出 settle，重新 PID 接近（允许倒车）
                            self._settle_start = None
                            self._settle_total_start = time.monotonic()  # ★ 重置总超时，给倒车独立时间窗口
                            raw_speed = error * MANEUVER['approach_kp_dist']
                            raw_speed = max(MANEUVER['approach_min_speed'],
                                            min(step.speed, abs(raw_speed)))
                            self.loop.set_target(raw_speed if error > 0 else -raw_speed,
                                                 step.turn_bias)
                            print(f"[FSM] APPROACH 偏离 dist={dist:.1f}cm "
                                  f"error={error:+.1f}cm，{'前进' if error > 0 else '倒车'}修正")
                        else:
                            # 噪声抖动，保持停车不动，settle 计时继续
                            self.loop.set_target(0, 0)
                            if time.monotonic() - self._settle_start >= MANEUVER['approach_settle_time']:
                                print(f"[FSM] APPROACH 稳定完成 dist={dist:.1f}cm ({step.desc})")
                                self._advance_step()
                    else:
                        # ── 正常 PID 接近 ──
                        raw_speed = error * MANEUVER['approach_kp_dist']
                        raw_speed = max(MANEUVER['approach_min_speed'],
                                        min(step.speed, abs(raw_speed)))

                        if error > MANEUVER['approach_stop_tol']:
                            # 太远：前进 PID 闭环
                            self.loop.set_target(raw_speed, step.turn_bias)
                        elif error < -MANEUVER['approach_stop_tol']:
                            # 冲过头：倒车（motor.py _loop 对负 base 自动用直通模式）
                            self.loop.set_target(-raw_speed, step.turn_bias)
                            print(f"[FSM] APPROACH 冲过头 dist={dist:.1f}cm "
                                  f"error={error:+.1f}cm，倒车修正  speed={raw_speed:.0f}")
                        else:
                            # |error| <= tol：首次到位，停车并开始 settle 计时
                            self.loop.set_target(0, 0)
                            self._settle_start = time.monotonic()
                            print(f"[FSM] APPROACH 到位 dist={dist:.1f}cm，"
                                  f"稳定 {MANEUVER['approach_settle_time']}s")
                else:
                    # 无超声波或未设 target_distance，回退固定速度 + 超时
                    self.loop.set_target(step.speed, step.turn_bias)
                    if self._elapsed() > step.timeout:
                        print(f"[FSM] APPROACH 超时 ({step.desc})")
                        self._advance_step()

                # 全局超时保底
                if self._elapsed() > step.timeout:
                    print(f"[FSM] APPROACH 超时保底 ({step.desc})")
                    self._advance_step()
                return True

            # ── STOP：短暂停车消除惯性 ──
            elif action == 'stop':
                self.loop.set_target(0, 0)
                if self._elapsed() > step.timeout:
                    print(f"[FSM] STOP 完成 ({step.desc})")
                    self._advance_step()
                return True

            # ── ROTATE：原地旋转（一正一反）──
            elif action == 'rotate':
                if step.turn_bias > 0:
                    # 右转/CW：左轮前进(+)，右轮反转(-)
                    self.loop.set_raw(step.speed, -step.speed)
                else:
                    # 左转/CCW：左轮反转(-)，右轮前进(+)
                    self.loop.set_raw(-step.speed, step.speed)

                # 编码器角度判定：脉冲之和 × 标定系数
                dr, dl = self._enc_delta()
                total_pulses = abs(dr) + abs(dl)
                angle = total_pulses * self._K_rotate

                if angle >= step.target_angle:
                    print(f"[FSM] ROTATE 完成：{angle:.1f}°/{step.target_angle}° "
                          f"pulses={total_pulses} ({step.desc})")
                    self.loop.set_target(0, 0)
                    self._advance_step()
                elif self._elapsed() > step.timeout:
                    print(f"[FSM] ROTATE 超时：{angle:.1f}°/{step.target_angle}° ({step.desc})")
                    self.loop.set_target(0, 0)
                    self._advance_step()
                return True

            # ── STRAIGHT：PID 闭环直行 ──
            #   target_distance>0 且 timeout>0 → 两者都满足才退出（AND）
            #   target_distance=0              → 纯时间判定
            #   超时 3 倍强制保底                 → 防止编码器故障无限等待
            elif action == 'straight':
                self.loop.set_target(step.speed, step.turn_bias)

                done = False

                # 条件 1：距离达标（默认 True，未设距离时直接满足）
                dist_met = True
                if step.target_distance > 0:
                    dr, dl = self._enc_delta()
                    avg_pulses = (abs(dr) + abs(dl)) / 2.0
                    distance_cm = avg_pulses / self._pulses_per_cm
                    dist_met = distance_cm >= step.target_distance

                # 条件 2：时间达标
                time_met = self._elapsed() >= step.timeout

                # ── 正常退出：距离和时间都满足 ──
                if dist_met and time_met:
                    if step.target_distance > 0:
                        dr, dl = self._enc_delta()
                        avg_pulses = (abs(dr) + abs(dl)) / 2.0
                        distance_cm = avg_pulses / self._pulses_per_cm
                        print(f"[FSM] STRAIGHT 完成：{distance_cm:.1f}cm/{step.target_distance}cm  "
                              f"t={self._elapsed():.1f}s/{step.timeout}s ({step.desc})")
                    else:
                        print(f"[FSM] STRAIGHT 时间达成 t={self._elapsed():.1f}s/"
                              f"{step.timeout}s ({step.desc})")
                    done = True

                # ── 超时 3 倍强制保底（编码器故障时不会无限等）──
                elif self._elapsed() > step.timeout * 3:
                    print(f"[FSM] STRAIGHT 超时保底 t={self._elapsed():.1f}s ({step.desc})")
                    done = True

                if done:
                    self._advance_step()
                return True

            # ── 未知 action ──
            else:
                print(f"[FSM] MANEUVER: 未知 action={action}，跳过")
                self._advance_step()
                return True

        # ════════════════════════════════════════════
        #  POST_WAIT — 完成魔方后等待，消除惯性（v3.1）
        # ════════════════════════════════════════════
        elif self.state == State.POST_WAIT:
            self.loop.set_target(0, 0)
            if self._elapsed() > MANEUVER['post_maneuver_wait']:
                print(f"[FSM] POST_WAIT 完成 ({MANEUVER['post_maneuver_wait']}s)，进入 SCAN_NEXT")
                self._enter(State.SCAN_NEXT)
            return True

        # ════════════════════════════════════════════
        #  SCAN_NEXT — 原地旋转扫描下一个魔方（v3.1）
        #  阶段：left（左转扫描）→ right（右转双倍扫描）
        #  纯摄像头检测，忽略刚通过的魔方颜色，任一阶段发现 → ALIGN_CUBE
        # ════════════════════════════════════════════
        elif self.state == State.SCAN_NEXT:

            # ── 阶段 1：原地左转扫描 ──
            if self._scan_next_phase == 'left':
                # 左转：右轮前进(+)，左轮反转(-)
                self.loop.set_raw(-MANEUVER['scan_next_speed'],
                                   MANEUVER['scan_next_speed'])

                color = self._try_detect_cube(frame, require_distance=False, dist=dist,
                                              ignore_color=self._last_passed_color)
                if color:
                    self.loop.set_target(0, 0)
                    self._next_color = color
                    self.target_color = color
                    print(f"[FSM] SCAN_NEXT 检测到 {color} 魔方，进入 ALIGN_CUBE")
                    self._enter(State.ALIGN_CUBE)
                    return True

                # 角度判定
                dr, dl = self._enc_delta()
                angle = (abs(dr) + abs(dl)) * self._K_rotate
                if angle >= MANEUVER['scan_next_angle']:
                    print(f"[FSM] SCAN_NEXT 左扫完成 ({angle:.1f}°)，进入右扫")
                    self.loop.set_target(0, 0)  # ★ 先停车，防止左转惯性带入右扫
                    self._scan_next_phase = 'right'
                    self._t0 = time.monotonic()
                    r, l = get_encoder_counts()
                    self._enc0_r = r
                    self._enc0_l = l
                elif self._elapsed() > MANEUVER['scan_next_timeout']:
                    # 超时保底：电机可能堵转，强制进入右扫
                    print(f"[FSM] SCAN_NEXT 左扫超时 "
                          f"({MANEUVER['scan_next_timeout']}s) angle={angle:.1f}°，"
                          f"强制进入右扫")
                    self.loop.set_target(0, 0)
                    self._scan_next_phase = 'right'
                    self._t0 = time.monotonic()
                    r, l = get_encoder_counts()
                    self._enc0_r = r
                    self._enc0_l = l
                return True

            # ── 阶段 2：原地右转扫描（双倍角度覆盖右侧+回中）──
            elif self._scan_next_phase == 'right':
                # 右转：左轮前进(+)，右轮反转(-)
                self.loop.set_raw(MANEUVER['scan_next_speed'],
                                  -MANEUVER['scan_next_speed'])

                color = self._try_detect_cube(frame, require_distance=False, dist=dist,
                                              ignore_color=self._last_passed_color)
                if color:
                    self.loop.set_target(0, 0)
                    self._next_color = color
                    self.target_color = color
                    print(f"[FSM] SCAN_NEXT 检测到 {color} 魔方，进入 ALIGN_CUBE")
                    self._enter(State.ALIGN_CUBE)
                    return True

                # 角度判定：双倍扫描覆盖右侧
                dr, dl = self._enc_delta()
                angle = (abs(dr) + abs(dl)) * self._K_rotate
                if angle >= MANEUVER['scan_next_angle'] * 2:
                    print(f"[FSM] SCAN_NEXT 扫描完毕未发现魔方，回退 SEARCH")
                    self._enter(State.SEARCH)
                elif self._elapsed() > MANEUVER['scan_next_timeout'] * 2:
                    # 超时保底：右扫双倍角度，超时翻倍
                    print(f"[FSM] SCAN_NEXT 右扫超时 "
                          f"({MANEUVER['scan_next_timeout'] * 2:.0f}s) angle={angle:.1f}°，"
                          f"强制回 SEARCH")
                    self._enter(State.SEARCH)
                return True

            # 未知阶段保护
            self._scan_next_phase = 'left'
            return True

        # ════════════════════════════════════════════
        #  ALIGN_CUBE — 两阶段对准（v3.9）
        #   阶段1 rotate：原地左右转，PD 控制对准摄像头中心
        #   阶段2 forward：对准后直线前进到停止距离
        # ════════════════════════════════════════════
        elif self.state == State.ALIGN_CUBE:

            # ── 阶段 1：原地旋转对准 ──
            if self._align_phase == 'rotate':
                turn_power = 0.0

                if frame is not None and self._next_color:
                    cx, _ = detect_cube(frame, self._next_color)
                    h, w = frame.shape[:2]
                    if cx is not None:
                        err = (cx - w / 2.0) / (w / 2.0)       # [-1, +1]，正=右，负=左
                        derr = err - self._align_prev_err
                        self._align_prev_err = err

                        if abs(err) < MANEUVER['align_cube_deadband']:
                            # 已对准！进入前进阶段
                            print(f"[FSM] ALIGN_CUBE 对准完成 err={err:+.3f}，开始直行前进")
                            self.loop.set_target(0, 0)
                            self._align_phase = 'forward'
                            self._t0 = time.monotonic()  # 重置超时，给 forward 阶段独立计时
                            return True
                        else:
                            # PD 控制：正=右转，负=左转
                            turn_power = (err * MANEUVER['align_cube_kp'] +
                                          derr * MANEUVER['align_cube_kd'])
                            turn_power = max(-MANEUVER['align_cube_rotate_speed'],
                                             min(MANEUVER['align_cube_rotate_speed'], turn_power))
                            # 确保最低速度克服静摩擦
                            if abs(turn_power) > 0 and abs(turn_power) < MANEUVER['approach_min_speed']:
                                turn_power = MANEUVER['approach_min_speed'] if turn_power > 0 else -MANEUVER['approach_min_speed']
                            print(f"[FSM] ALIGN_CUBE 旋转对准  cx={cx}  err={err:+.3f}  "
                                  f"turn_power={turn_power:+.1f}")
                    else:
                        # 丢失目标：慢速继续上次方向旋转搜索
                        last_dir = 1.0 if self._align_prev_err > 0 else -1.0
                        turn_power = last_dir * MANEUVER['approach_min_speed']
                        print(f"[FSM] ALIGN_CUBE 丢失目标，慢速{'右' if last_dir > 0 else '左'}转搜索")
                else:
                    # 无帧或无颜色：慢速右转搜索
                    turn_power = MANEUVER['approach_min_speed']

                # 输出原地旋转：右转=左进右退，左转=左退右进
                self.loop.set_raw(turn_power, -turn_power)

                # 旋转超时保底：超时后直接进入前进阶段
                if self._elapsed() > MANEUVER['align_cube_rotate_timeout']:
                    print(f"[FSM] ALIGN_CUBE 旋转超时 ({MANEUVER['align_cube_rotate_timeout']}s)，"
                          f"直接前进")
                    self.loop.set_target(0, 0)
                    self._align_phase = 'forward'
                    self._t0 = time.monotonic()
                return True

            # ── 阶段 2：直线前进接近 ──
            elif self._align_phase == 'forward':
                self.loop.set_target(MANEUVER['align_cube_speed'], 0.0)

                # 退出条件 1：超声波距离达标
                if dist is not None and dist <= MANEUVER['align_cube_stop_dist']:
                    print(f"[FSM] ALIGN_CUBE 到达停止距离 {dist:.1f}cm，进入 SEARCH")
                    self.loop.set_target(0, 0)
                    self._enter(State.SEARCH)
                # 退出条件 2：forward 阶段超时
                elif self._elapsed() > MANEUVER['align_cube_timeout']:
                    print(f"[FSM] ALIGN_CUBE 前进超时 ({MANEUVER['align_cube_timeout']}s)，进入 SEARCH")
                    self._enter(State.SEARCH)
                return True

            # 未知阶段保护
            self._align_phase = 'rotate'
            return True

        # ════════════════════════════════════════════
        #  红色魔方专用状态（保持不变）
        # ════════════════════════════════════════════

        # ════ ALIGN_RED（摄像头比例对准） ════
        elif self.state == State.ALIGN_RED:
            if frame is not None:
                cx, _ = detect_cube(frame, 'red')
                h, w  = frame.shape[:2]
                if cx is not None:
                    err       = (cx - w / 2.0) / (w / 2.0)
                    turn_bias = err * MANEUVER['align_centering_kp']
                    self.loop.set_target(MANEUVER['align_red_speed'], turn_bias)
                else:
                    self.loop.set_target(MOTOR['base_speed'] * 0.7, 0.0)
            else:
                self.loop.set_target(MOTOR['base_speed'] * 0.7, 0.0)

            threshold = MANEUVER['orbit_radius'] + MANEUVER['approach_margin'] + 10
            if dist is not None and dist < threshold:
                print(f"[FSM] ALIGN_RED: 距离 {dist:.1f}cm < {threshold}cm，进入 APPROACH")
                self._enter(State.APPROACH)
            elif self._elapsed() > MANEUVER['align_red_timeout']:
                print("[FSM] ALIGN_RED 超时，重新 SEARCH")
                self._enter(State.SEARCH)
            return True

        # ════ APPROACH（红色专用：距离PID停车到入轨距离） ════
        elif self.state == State.APPROACH:
            target_dist = MANEUVER['orbit_radius']
            if dist is not None and target_dist > 0:
                error = dist - target_dist   # >0 太远需前进，<0 太近需倒车
                settling = self._settle_start is not None

                # ── 首次进入时启动总超时计时（仅一次）──
                if self._settle_total_start is None:
                    self._settle_total_start = time.monotonic()

                # ── settle 总超时保护 ──
                if time.monotonic() - self._settle_total_start > MANEUVER['red_approach_settle_timeout']:
                    print(f"[FSM] APPROACH(红) 总超时 "
                          f"({MANEUVER['red_approach_settle_timeout']}s)，强制进入 TANGENT")
                    self._settle_start = None
                    self._settle_total_start = None
                    self._enter(State.TANGENT)
                    return True

                if settling:
                    # ── 已在 settle 状态 ──
                    if abs(error) > MANEUVER['approach_stop_tol'] * 2:
                        # 显著偏离，退出 settle，重新 PID 接近
                        self._settle_start = None
                        self._settle_total_start = time.monotonic()  # ★ 重置总超时，给倒车独立时间窗口
                        raw_speed = error * MANEUVER['approach_kp_dist']
                        raw_speed = max(MANEUVER['approach_min_speed'],
                                        min(MANEUVER['approach_speed'], abs(raw_speed)))
                        self.loop.set_target(raw_speed if error > 0 else -raw_speed, 0.0)
                        print(f"[FSM] APPROACH(红) 偏离 dist={dist:.1f}cm "
                              f"error={error:+.1f}cm，{'前进' if error > 0 else '倒车'}修正")
                    else:
                        # 噪声抖动，保持停车，settle 计时继续
                        self.loop.set_target(0, 0)
                        if time.monotonic() - self._settle_start >= MANEUVER['red_approach_settle_time']:
                            print(f"[FSM] APPROACH(红) 稳定完成 dist={dist:.1f}cm，进入 TANGENT")
                            self._settle_start = None
                            self._settle_total_start = None
                            self._enter(State.TANGENT)
                else:
                    # ── 正常 PID 接近 ──
                    raw_speed = error * MANEUVER['approach_kp_dist']
                    raw_speed = max(MANEUVER['approach_min_speed'],
                                    min(MANEUVER['approach_speed'], abs(raw_speed)))

                    if error > MANEUVER['approach_stop_tol']:
                        # 太远：前进 PID 闭环
                        self.loop.set_target(raw_speed, 0.0)
                    elif error < -MANEUVER['approach_stop_tol']:
                        # 冲过头：倒车
                        self.loop.set_target(-raw_speed, 0.0)
                        print(f"[FSM] APPROACH(红) 冲过头 dist={dist:.1f}cm "
                              f"error={error:+.1f}cm，倒车修正  speed={raw_speed:.0f}")
                    else:
                        # |error| <= tol：首次到位，停车并开始 settle 计时
                        self.loop.set_target(0, 0)
                        self._settle_start = time.monotonic()
                        print(f"[FSM] APPROACH(红) 到位 dist={dist:.1f}cm，"
                              f"稳定 {MANEUVER['red_approach_settle_time']}s")
            else:
                # 无超声波回退固定速度 + 超时
                self.loop.set_target(MANEUVER['approach_speed'], 0.0)
                if self._elapsed() > MANEUVER['approach_timeout']:
                    print(f"[FSM] APPROACH(红) 超时保底（无超声波），强制进入 TANGENT")
                    self._settle_start = None
                    self._settle_total_start = None
                    self._enter(State.TANGENT)

            # 全局超时保底
            if self._elapsed() > MANEUVER['approach_timeout']:
                print(f"[FSM] APPROACH(红) 超时保底，强制进入 TANGENT")
                self._settle_start = None
                self._settle_total_start = None
                self._enter(State.TANGENT)
            return True

        # ════ TANGENT（v3.1：先停再原地右转，与黄/绿 rotate 一致）════
        elif self.state == State.TANGENT:

            # ── 阶段 1：短暂停车消除惯性 ──
            if self._tangent_phase == 'stop':
                self.loop.set_target(0, 0)
                if self._elapsed() > MANEUVER['stop_time']:
                    print(f"[FSM] TANGENT 停车完成 ({MANEUVER['stop_time']}s)，开始原地右转")
                    self._tangent_phase = 'rotate'
                    # 重置编码器基准，用于测量旋转角度
                    r, l = get_encoder_counts()
                    self._enc0_r = r
                    self._enc0_l = l
                    self._t0 = time.monotonic()  # 重置超时计时
                return True

            # ── 阶段 2：原地右转（左轮前进+，右轮反转-）──
            elif self._tangent_phase == 'rotate':
                # 右转/CW：左轮前进，右轮反转
                self.loop.set_raw(MANEUVER['tangent_speed'],
                                  -MANEUVER['tangent_speed'])

                # 编码器角度判定：脉冲之和 × 标定系数（与黄/绿 rotate 一致）
                dr, dl = self._enc_delta()
                total_pulses = abs(dr) + abs(dl)
                angle = total_pulses * self._K_rotate

                print(f"[FSM] TANGENT  angle={angle:.1f}°/{MANEUVER['tangent_target_deg']}°  "
                      f"pulses={total_pulses}  t={self._elapsed():.1f}s")

                if angle >= MANEUVER['tangent_target_deg']:
                    print(f"[FSM] TANGENT 完成：{angle:.1f}° 切向对齐，进入 ORBIT")
                    self.loop.set_target(0, 0)
                    self._enter(State.ORBIT)
                elif self._elapsed() > MANEUVER['tangent_timeout']:
                    print(f"[FSM] TANGENT 超时：{angle:.1f}°/{MANEUVER['tangent_target_deg']}°，"
                          f"强制进入 ORBIT")
                    self.loop.set_target(0, 0)
                    self._enter(State.ORBIT)
                return True

            # 未知阶段保护
            self._tangent_phase = 'stop'
            return True

        # ════ ORBIT（固定差速弧线逆时针绕行） ════
        elif self.state == State.ORBIT:
            self.loop.set_target(MANEUVER['orbit_speed'],
                                 MANEUVER['orbit_turn_bias'])

            angle = -self._rot_angle()
            print(f"[FSM] ORBIT  angle={angle:+.1f}°/{MANEUVER['orbit_target_deg']}°  "
                  f"t={self._elapsed():.1f}s")

            if abs(angle) >= MANEUVER['orbit_target_deg']:
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
            elif self._elapsed() > MANEUVER['orbit_timeout']:
                print("[FSM] ORBIT 超时，强制退出")
                self._enter(State.RECOVER_RED)
            return True

        # ════ RECOVER_RED（红色专用回正） ════
        elif self.state == State.RECOVER_RED:
            self.loop.set_target(MANEUVER['red_recover_speed'],
                                 -MANEUVER['red_recover_turn'])

            if self._elapsed() > MANEUVER['red_recover_time']:
                self.cubes_passed += 1
                print(f"[FSM] 红色绕行完成，已通过 {self.cubes_passed}/3 个魔方")
                if self.cubes_passed >= 3:
                    self._enter(State.FINISH)
                else:
                    # v3.1：记录刚通过的颜色，SCAN_NEXT 中忽略它
                    self._last_passed_color = 'red'
                    self._next_color = None
                    self._enter(State.POST_WAIT)
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
