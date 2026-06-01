"""
魔方绕桩小车 —— 电机驱动模块
负责：GPIO初始化、PWM输出、PID速度环、move() 差速接口

硬件：L298N + 霍尔编码器 + 树莓派 (BCM 编号)
注意：I1/I4 高电平前进，I2/I3 高电平为反转（刹车用）
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


# ==================== 底层电机控制 ====================

def _set_right(duty: float):
    """设置右轮占空比（正向前进）。duty=0 停止。"""
    duty = max(0.0, min(100.0, duty))
    if duty < 0.5:
        GPIO.output(PIN_IN1, GPIO.LOW)
        GPIO.output(PIN_IN2, GPIO.LOW)
        _pwm_r.ChangeDutyCycle(0)
    else:
        GPIO.output(PIN_IN1, GPIO.HIGH)  # IN1高 / IN2低 → 右轮前进
        GPIO.output(PIN_IN2, GPIO.LOW)
        _pwm_r.ChangeDutyCycle(duty)


def _set_left(duty: float):
    """设置左轮占空比（正向前进）。duty=0 停止。"""
    duty = max(0.0, min(100.0, duty))
    if duty < 0.5:
        GPIO.output(PIN_IN3, GPIO.LOW)
        GPIO.output(PIN_IN4, GPIO.LOW)
        _pwm_l.ChangeDutyCycle(0)
    else:
        GPIO.output(PIN_IN4, GPIO.HIGH)  # IN4高 / IN3低 → 左轮前进
        GPIO.output(PIN_IN3, GPIO.LOW)
        _pwm_l.ChangeDutyCycle(duty)


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
    GPIO.output(PIN_IN3, GPIO.HIGH)   # ← 原版错误：此处为 LOW，与前进相同
    GPIO.output(PIN_IN4, GPIO.LOW)    # ← 原版错误：此处为 HIGH，与前进相同
    _pwm_r.ChangeDutyCycle(50)
    _pwm_l.ChangeDutyCycle(50)
    time.sleep(duration)
    stop()


# ==================== PID 速度环 ====================

class PIDSpeedLoop:
    """
    双轮 PID 速度闭环，以独立线程定时运行。
    """

    PULSES_PER_REV = 585

    def __init__(self):
        cfg_l = MOTOR['pid_left']
        cfg_r = MOTOR['pid_right']
        self._pid_l = PIDController(**cfg_l)
        self._pid_r = PIDController(**cfg_r)
        self._period = MOTOR['pid_period']

        self._target_speed: float = 0.0
        self._turn_bias:    float = 0.0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        self._last_r = 0
        self._last_l = 0

    def set_target(self, speed: float, turn_bias: float = 0.0):
        turn_bias = max(-1.0, min(1.0, turn_bias))
        with self._lock:
            self._target_speed = speed
            self._turn_bias    = turn_bias

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

            speed_r = delta_r / self.PULSES_PER_REV
            speed_l = delta_l / self.PULSES_PER_REV

            with self._lock:
                base = self._target_speed
                bias = self._turn_bias

            if base < 0.5:
                stop()
                time.sleep(self._period)
                continue

            base_ideal_l = self._pid_l.setpoint
            base_ideal_r = self._pid_r.setpoint

            factor_l = max(min_duty / 100.0, 1.0 + bias * turn_rate)
            factor_r = max(min_duty / 100.0, 1.0 - bias * turn_rate)

            self._pid_l.setpoint = base_ideal_l * factor_l * (base / MOTOR['base_speed'])
            self._pid_r.setpoint = base_ideal_r * factor_r * (base / MOTOR['base_speed'])

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
