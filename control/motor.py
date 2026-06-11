"""
魔方绕桩小车 —— 电机驱动模块
负责：GPIO初始化、PWM输出、PID速度环、move() 差速接口

硬件：L298N + 霍尔编码器 + 树莓派 (BCM 编号)
注意：I1/I4 高电平前进，I2/I3 高电平为反转（刹车用）

v2 更新：
  • _set_left / _set_right 支持负 duty（反转），用于原地旋转
  • PIDSpeedLoop 增加 set_raw() 直通模式，原地旋转时绕过 PID
  • 修复 PID setpoint 指数漂移 bug（用固定标定值 rated_speed 换算）
"""

import os
import sys
import time
import threading
from typing import Optional

import RPi.GPIO as GPIO

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import MOTOR
from sensors.encoder import get_encoder_counts, reset_encoder_counts, init as encoder_init, cleanup as encoder_cleanup
from control.pid import PIDController

# ==================== GPIO 引脚定义 ====================
# 右电机
PIN_ENA = 13   # PWM 使能
PIN_IN1 = 26   # 方向 A（高电平 = 右轮前进）
PIN_IN2 = 19   # 方向 B

# 左电机
PIN_ENB = 16   # PWM 使能
PIN_IN3 = 21   # 方向 A
PIN_IN4 = 20   # 方向 B（高电平 = 左轮前进）

# PWM 频率（Hz）
PWM_FREQ = 1000

_pwm_r = None  # type: Optional[GPIO.PWM]
_pwm_l = None  # type: Optional[GPIO.PWM]

_loop_active = False


# ==================== 初始化 / 清理 ====================

def init():
    global _pwm_r, _pwm_l
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for pin in (PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4):
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_ENA, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_ENB, GPIO.OUT, initial=GPIO.LOW)
    _pwm_r = GPIO.PWM(PIN_ENA, PWM_FREQ)
    _pwm_l = GPIO.PWM(PIN_ENB, PWM_FREQ)
    _pwm_r.start(0)
    _pwm_l.start(0)
    encoder_init()
    print("[motor] 初始化完成")


def cleanup():
    global _loop_active
    _loop_active = False
    stop()
    encoder_cleanup()
    try:
        if _pwm_r:
            _pwm_r.stop()
        if _pwm_l:
            _pwm_l.stop()
        GPIO.cleanup()
    except Exception:
        pass
    print("[motor] GPIO 已释放")


# ==================== 底层电机控制（支持反转） ====================

def _set_right(duty: float):
    """
    设置右轮占空比。
    duty > 0  →  前进（IN1=HIGH, IN2=LOW）
    duty < 0  →  反转（IN1=LOW, IN2=HIGH）
    duty ≈ 0  →  停止
    """
    duty_abs = abs(duty)
    if duty_abs < 0.5:
        GPIO.output(PIN_IN1, GPIO.LOW)
        GPIO.output(PIN_IN2, GPIO.LOW)
        _pwm_r.ChangeDutyCycle(0)
    elif duty > 0:
        GPIO.output(PIN_IN1, GPIO.HIGH)   # 前进
        GPIO.output(PIN_IN2, GPIO.LOW)
        _pwm_r.ChangeDutyCycle(min(100.0, duty_abs))
    else:
        GPIO.output(PIN_IN1, GPIO.LOW)    # 反转
        GPIO.output(PIN_IN2, GPIO.HIGH)
        _pwm_r.ChangeDutyCycle(min(100.0, duty_abs))


def _set_left(duty: float):
    """
    设置左轮占空比。
    duty > 0  →  前进（IN4=HIGH, IN3=LOW）
    duty < 0  →  反转（IN3=HIGH, IN4=LOW）
    duty ≈ 0  →  停止
    """
    duty_abs = abs(duty)
    if duty_abs < 0.5:
        GPIO.output(PIN_IN3, GPIO.LOW)
        GPIO.output(PIN_IN4, GPIO.LOW)
        _pwm_l.ChangeDutyCycle(0)
    elif duty > 0:
        GPIO.output(PIN_IN4, GPIO.HIGH)   # 前进
        GPIO.output(PIN_IN3, GPIO.LOW)
        _pwm_l.ChangeDutyCycle(min(100.0, duty_abs))
    else:
        GPIO.output(PIN_IN3, GPIO.HIGH)   # 反转
        GPIO.output(PIN_IN4, GPIO.LOW)
        _pwm_l.ChangeDutyCycle(min(100.0, duty_abs))


# ==================== 高层差速接口 ====================

def move(speed: float, turn_bias: float = 0.0):
    """
    差速驱动接口。

    参数
    ----
    speed      : 基准占空比 0-100。
    turn_bias  : 转向偏置 -1.0 ~ +1.0。
                 负值 = 左转（左轮慢、右轮快）
                 正值 = 右转（右轮慢、左轮快）
    """
    turn_bias = max(-1.0, min(1.0, turn_bias))

    global _loop_active
    if _loop_active:
        print("[motor] 警告：PIDSpeedLoop 正在运行，move() 被忽略。请先 loop.stop()")
        return

    turn_rate = MOTOR['turn_rate']
    min_duty  = MOTOR['min_duty']

    left_duty  = speed * (1.0 + turn_bias * turn_rate)
    right_duty = speed * (1.0 - turn_bias * turn_rate)

    # 硬限幅：确保两轮始终正转
    left_duty  = max(float(min_duty), min(100.0, left_duty))
    right_duty = max(float(min_duty), min(100.0, right_duty))

    _set_left(left_duty)
    _set_right(right_duty)


def stop():
    """立即停止两轮。"""
    _set_left(0)
    _set_right(0)


def brake(duration: float = 0.1):
    """
    短暂制动后停止（对调方向引脚产生反电动势制动）。

    修复说明：原版左轮制动方向与前进方向相同（无效），现已修正：
      右轮制动：IN1=LOW, IN2=HIGH（与前进相反）
      左轮制动：IN3=HIGH, IN4=LOW（与前进相反）
    """
    global _loop_active
    if _loop_active:
        print("[motor] 警告：PIDSpeedLoop 正在运行，brake() 被忽略。请先 loop.stop()")
        return

    # 右轮：前进是 IN1=HIGH/IN2=LOW，制动反转
    GPIO.output(PIN_IN1, GPIO.LOW)
    GPIO.output(PIN_IN2, GPIO.HIGH)
    # 左轮：前进是 IN4=HIGH/IN3=LOW，制动反转（原版此处有 bug，已修正）
    GPIO.output(PIN_IN3, GPIO.HIGH)
    GPIO.output(PIN_IN4, GPIO.LOW)
    _pwm_r.ChangeDutyCycle(50)
    _pwm_l.ChangeDutyCycle(50)
    time.sleep(duration)
    stop()


# ==================== PID 速度环 ====================

class PIDSpeedLoop:
    """
    双轮 PID 速度闭环，以独立线程定时运行。

    v2 更新：
      • set_raw() — 直通模式，绕过 PID 直接设置两轮 duty（用于原地旋转）
      • 修复 setpoint 指数漂移：固定标定转速 rated_speed 作为换算基准
    """

    PULSES_PER_REV = 585

    def __init__(self):
        cfg_l = MOTOR['pid_left']
        cfg_r = MOTOR['pid_right']
        self._pid_l = PIDController(Kp=cfg_l['Kp'], Ki=cfg_l['Ki'], Kd=cfg_l['Kd'],
                                     rated_speed=MOTOR['rated_speed_l'])
        self._pid_r = PIDController(Kp=cfg_r['Kp'], Ki=cfg_r['Ki'], Kd=cfg_r['Kd'],
                                     rated_speed=MOTOR['rated_speed_r'])
        self._period = MOTOR['pid_period']

        self._target_speed: float = 0.0
        self._turn_bias:    float = 0.0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        self._last_r = 0
        self._last_l = 0

        # ★ 保存标定转速作为 setpoint 计算基准
        #   修复：原代码用 self._pid.setpoint（会被指数级污染）
        self._rated_speed_l = MOTOR['rated_speed_l']
        self._rated_speed_r = MOTOR['rated_speed_r']

        # 直通模式（原地旋转时绕过 PID）
        self._passthrough = False
        self._raw_left:  float = 0.0
        self._raw_right: float = 0.0

    def set_target(self, speed: float, turn_bias: float = 0.0):
        """PID 闭环控制（正常行驶）。"""
        turn_bias = max(-1.0, min(1.0, turn_bias))
        with self._lock:
            self._passthrough = False
            self._target_speed = speed
            self._turn_bias    = turn_bias

    def set_raw(self, left_duty: float, right_duty: float):
        """
        直通模式：绕过 PID，直接设置两轮占空比。
        用于原地旋转（一正一反）或精确刹车。
        正值 = 前进，负值 = 反转。
        """
        with self._lock:
            self._passthrough = True
            self._raw_left  = left_duty
            self._raw_right = right_duty

    def start(self):
        global _loop_active
        if self._running:
            return
        self._running = True
        _loop_active = True
        self._pid_l.reset()
        self._pid_r.reset()
        reset_encoder_counts()
        self._last_r = 0
        self._last_l = 0
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        global _loop_active
        self._running = False
        _loop_active = False
        if self._thread:
            self._thread.join(timeout=1.0)
        stop()

    def _loop(self):
        min_duty  = float(MOTOR['min_duty'])
        turn_rate = MOTOR['turn_rate']

        while self._running:
            t0 = time.monotonic()

            cur_r, cur_l = get_encoder_counts()
            delta_r = cur_r - self._last_r
            delta_l = cur_l - self._last_l
            self._last_r = cur_r
            self._last_l = cur_l

            # ── 读取当前控制模式（快进快出，不在锁内sleep）──
            with self._lock:
                passthrough = self._passthrough
                raw_left    = self._raw_left
                raw_right   = self._raw_right
                base = self._target_speed
                bias = self._turn_bias

            # ── 直通模式：直接输出 raw duty（原地旋转 / 倒车）──
            if passthrough:
                _set_left(raw_left)
                _set_right(raw_right)
                elapsed = time.monotonic() - t0
                sleep_t = self._period - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)
                continue

            # ── 停止：|base| 极小 ──
            if abs(base) < 0.5:
                stop()
                elapsed = time.monotonic() - t0
                sleep_t = self._period - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)
                continue

            # ── 倒车：编码器不区分方向，无法 PID 闭环，用直通方式输出负占空比 ──
            if base < 0:
                rev_base = abs(base)
                # turn_bias 方向含义在倒车时保持一致：
                #   bias>0 → 右侧慢（右转）→ 倒车时左轮更负
                factor_l = max(min_duty / 100.0, 1.0 + bias * turn_rate)
                factor_r = max(min_duty / 100.0, 1.0 - bias * turn_rate)
                out_l = -rev_base * factor_l
                out_r = -rev_base * factor_r
                # 钳制到 [-100, -min_duty]
                out_l = max(-100.0, min(-min_duty, out_l))
                out_r = max(-100.0, min(-min_duty, out_r))
                _set_left(out_l)
                _set_right(out_r)
                elapsed = time.monotonic() - t0
                sleep_t = self._period - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)
                continue

            # ── 前进：PID 闭环 ──
            speed_r = delta_r / self.PULSES_PER_REV
            speed_l = delta_l / self.PULSES_PER_REV

            factor_l = max(min_duty / 100.0, 1.0 + bias * turn_rate)
            factor_r = max(min_duty / 100.0, 1.0 - bias * turn_rate)

            # ★ 占空比→转速换算：rated_speed × 转向因子 × (实际占空比/基准占空比)
            self._pid_l.setpoint = self._rated_speed_l * factor_l * (base / MOTOR['base_speed'])
            self._pid_r.setpoint = self._rated_speed_r * factor_r * (base / MOTOR['base_speed'])

            ff_l = base * factor_l
            ff_r = base * factor_r

            out_l = ff_l + self._pid_l.compute(speed_l)
            out_r = ff_r + self._pid_r.compute(speed_r)

            out_l = max(min_duty, min(100.0, out_l))
            out_r = max(min_duty, min(100.0, out_r))

            _set_left(out_l)
            _set_right(out_r)

            elapsed = time.monotonic() - t0
            sleep_t = self._period - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)


# ==================== 便捷工具函数 ====================

def move_timed(speed: float, turn_bias: float, duration: float):
    """以给定速度和转向偏置运动固定时间后停止（阻塞）。"""
    move(speed, turn_bias)
    time.sleep(duration)
    stop()


def forward(speed: Optional[float] = None):
    """直线前进。"""
    if speed is None:
        speed = MOTOR['base_speed']
    move(speed, 0.0)


# ==================== 自检 ====================

if __name__ == '__main__':
    print("=== motor.py 自检：双轮低速前进 2 秒 ===")
    try:
        init()
        reset_encoder_counts()
        forward()
        time.sleep(2.0)
        stop()
        r, l = get_encoder_counts()
        print(f"编码器：右轮 {r} 脉冲，左轮 {l} 脉冲")
        print(f"（585 脉冲/圈 → 右轮 {r/585:.2f} 圈，左轮 {l/585:.2f} 圈）")
    finally:
        cleanup()
