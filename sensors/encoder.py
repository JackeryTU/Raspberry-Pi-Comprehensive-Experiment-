"""
sensors/encoder.py
霍尔编码器驱动 —— 脉冲计数与里程计接口

注意：霍尔编码器正常转速下输出干净数字方波，不需要软件消抖。
"""

import threading
from typing import Tuple
import RPi.GPIO as GPIO

# ==================== GPIO 引脚定义 ====================
PIN_ENC_R = 12  # 右编码器（霍尔传感器）
PIN_ENC_L = 6   # 左编码器（霍尔传感器）

# ==================== 编码器计数（线程安全） ====================
_enc_lock = threading.Lock()
_enc_count_r = 0
_enc_count_l = 0


def _isr_right(channel):
    global _enc_count_r
    with _enc_lock:
        _enc_count_r += 1


def _isr_left(channel):
    global _enc_count_l
    with _enc_lock:
        _enc_count_l += 1


def init():
    """初始化编码器引脚和中断。由 motor.py 的 init() 调用。"""
    GPIO.setup(PIN_ENC_R, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(PIN_ENC_L, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(PIN_ENC_R, GPIO.RISING, callback=_isr_right)
    GPIO.add_event_detect(PIN_ENC_L, GPIO.RISING, callback=_isr_left)


def cleanup():
    """清理编码器中断。由 motor.py 的 cleanup() 调用。"""
    try:
        GPIO.remove_event_detect(PIN_ENC_R)
        GPIO.remove_event_detect(PIN_ENC_L)
    except Exception:
        pass


def get_encoder_counts() -> Tuple[int, int]:
    """返回 (右轮脉冲数, 左轮脉冲数) 的快照（不清零）。"""
    with _enc_lock:
        return _enc_count_r, _enc_count_l


def reset_encoder_counts():
    """将两路编码器计数清零（用于里程计重置）。"""
    global _enc_count_r, _enc_count_l
    with _enc_lock:
        _enc_count_r = 0
        _enc_count_l = 0
