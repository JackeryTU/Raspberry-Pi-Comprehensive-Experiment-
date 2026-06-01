"""
魔方绕桩小车 —— 全局参数配置
所有可调参数集中在此，修改时请注释原因和日期。
"""

import os
import json

# ==================== 摄像头配置 ====================
CAMERA = {
    'id': 0,              # USB摄像头设备号
    'width': 320,         # 捕获宽度（低分辨率减轻树莓派压力）
    'height': 240,        # 捕获高度
    'fps': 30,            # 请求帧率（实际可能略低）
    'warmup_frames': 5,   # 启动时丢弃的帧数（稳定自动曝光）
}

# ==================== 视觉处理配置 ====================
VISION = {
    # ROI：只处理图像下半部分，过滤天花板/远处干扰，减少计算量
    # 取值 0.0~1.0，表示从顶部开始保留的比例（0.3 = 只处理下方70%）
    'roi_y_start': 0.3,

    # 最小轮廓面积，小于此值视为噪点
    'min_contour_area': 500,

    # 形态学开运算核大小（去噪用，3x3足够轻量）
    'morph_kernel': 3,
}

# ==================== HSV 颜色阈值 (默认) ====================
# 重要：以下阈值需根据实际赛道光照条件微调！
# 建议在树莓派上运行 tools/calibrate_hsv.py 实时标定。
# OpenCV中 H: 0-179, S/V: 0-255
COLOR_THRESHOLDS = {
    'red': {
        'ranges': [
            [[0, 100, 100], [5, 255, 255]],
            [[165, 100, 100], [180, 255, 255]],
        ],
    },
    'yellow': {
        'ranges': [
            [[20, 100, 100], [35, 255, 255]],
        ],
    },
    'green': {
        'ranges': [
            [[35, 100, 100], [85, 255, 255]],
        ],
    },
}

CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), 'calibration_hsv.json')


def get_color_thresholds():
    """
    获取颜色阈值。优先从 calibration_hsv.json 加载，失败则返回默认值。
    """
    if os.path.exists(CALIBRATION_FILE):
        try:
            with open(CALIBRATION_FILE, 'r', encoding='utf-8') as f:
                loaded = json.load(f)
            for c in ['red', 'yellow', 'green']:
                if c not in loaded:
                    raise ValueError(f"校准文件缺少颜色: {c}")
            return loaded
        except Exception as e:
            print(f"[!] 加载校准文件失败 ({CALIBRATION_FILE}): {e}")
            print("[!] 回退到 config.py 中的默认阈值。")
    return COLOR_THRESHOLDS


# ==================== 电机与PID配置 ====================
MOTOR = {
    'base_speed': 40,      # 直线搜索基准占空比
    'min_duty': 5,         # 最低占空比，防止电机 stall
    'turn_rate': 0.6,      # 转向比例系数

    'pid_left':  {'Kp': 25, 'Ki': 0.01, 'Kd': 35, 'ideal_speed': 0.61},
    'pid_right': {'Kp': 25, 'Ki': 0.01, 'Kd': 40, 'ideal_speed': 0.57},
    'pid_period': 0.1,
}

# ==================== 机动动作参数 ====================
MANEUVER = {
    # ---------- 黄/绿 通过动作 ----------
    'align_speed':   35,
    'align_turn':    0.7,   # 切向转向偏置（正=右转, 负=左转，由状态机控制符号）
    'align_time':    0.4,   # 切向转向持续时间（秒）

    'pass_speed':    30,
    'pass_turn':     0.2,   # 侧身时轻微同向偏置
    'pass_time':     1.0,

    'recover_speed': 35,
    'recover_turn':  0.6,   # 回正偏置（由状态机控制符号）
    'recover_time':  0.4,

    # ---------- 红色魔方：切向入轨绕行360° ----------
    #
    # 策略：ALIGN（对准）→ APPROACH（直线接近）→ TANGENT（原地右转90°）→ ORBIT（弧线绕行）
    #
    # ALIGN（摄像头比例对准）
    'align_centering_kp':  0.4,   # 摄像头中心对准P增益（cx偏差→转向偏置）
    'align_red_speed':     30,    # 对准阶段前进速度
    'align_red_timeout':   6.0,   # 对准阶段超时（秒）

    # APPROACH（直线接近到入轨位置）
    # 接近目标距离 = orbit_radius（超声波测距）
    'approach_speed':      30,    # 接近速度
    'approach_timeout':    5.0,   # 接近超时（秒）

    # TANGENT（原地右转90°以切向入轨）
    # 物理含义：转完90°后，魔方位于车身正右侧，车头朝向绕圈切线方向
    # 注意：此处使用差速右转（左轮快，右轮慢），两轮均正转（绝不倒转）
    'tangent_speed':       18,    # 差速右转速度（基准占空比）
    'tangent_turn_bias':   0.85,  # 高右偏置，产生较紧的原地右转
    'tangent_target_deg':  90,    # 目标转角（度），需用 cal_angle.py 验证K值准确性
    'tangent_timeout':     3.0,   # 原地转向超时（秒）

    # 入轨半径：小车中心到魔方中心的距离（cm）
    # 设为25cm可留足安全余量；若魔方较大可适当增大
    'orbit_radius':        25,
    # 接近阶段目标：dist < orbit_radius + approach_margin 时停止前进，准备转向
    'approach_margin':     5,

    # ORBIT（弧线绕行）
    # 固定差速右转弧线：左轮快（外弧）右轮慢（内弧），保持入轨半径近似不变
    # 实际半径由 (orbit_speed, orbit_turn_bias) 决定，需实测标定
    'orbit_speed':         25,
    'orbit_turn_bias':     0.5,   # 右转偏置（越大 = 圆圈越小）
    'orbit_timeout':       8.0,   # 绕行超时保护（秒）

    # 编码器转角标定系数（度/脉冲差）
    # ★★ 必须用 cal_angle.py 在实车上标定！默认0.2大概率不准 ★★
    # 标定方法见 cal_angle.py 注释；标定时使用与ORBIT相同的speed/turn_bias
    'deg_per_pulse_diff':  0.2,
}

# ==================== 超声波触发阈值 ====================
ULTRASONIC = {
    'detect_dist': 60,   # 发现魔方距离（cm），进入 ALIGN
    'action_dist': 30,   # 动作触发距离（cm），进入 PASS / 开始 APPROACH
    'clear_dist':  50,   # 脱离判定距离（cm），认为已绕过魔方
}

# ==================== KS103 超声波配置 ====================
KS103 = {
    'bus':                    1,
    'address':                0x74,
    'mode':                   0xb0,
    'filter_window':          5,
    'outlier_threshold_cm':   15,
    'read_interval':          0.035,
}
