# 🚗 Cube Car — 魔方绕桩小车

> **Vibe Coding Ready** | 电子系统导论综合实验 | 树莓派 + Python

---

## 🎯 项目速览

| 项目 | 说明 |
|:---|:---|
| **任务** | 自动识别黄/红/绿三色魔方，按规则绕行/通过 |
| **平台** | 树莓派 + L298N + 霍尔编码器 + USB摄像头 + KS103超声波 |
| **语言** | Python 3 |
| **版本** | v3.10 — 统一占空比接口 + 倒车/扫描超时修复 + config 模块化重组 |
| **控制频率** | 10~20 Hz 主循环 |
| **核心策略** | 状态机 + PID速度环 + 距离P控制 + 编码器角度闭环 + 超声/面积融合 |

### 三色规则

| 颜色 | 规则 | 动作 |
|:---|:---|:---|
| 🟡 **黄色** | 从**左侧**通过 | 距离P接近 → 停车 → 原地左转 → 直行通过 → 原地右转回正 → 直行恢复 |
| 🔴 **红色** | **逆时针绕行 360°** | P对准 → 距离P接近入轨 → 原地右转切向 → 弧线绕行 → 回正 |
| 🟢 **绿色** | 从**右侧**通过 | 距离P接近 → 停车 → 原地右转 → 直行通过 → 原地左转回正 → 直行恢复 |

---

## 🏗️ 项目结构

```
cube_car/
├── main.py                    # 主程序入口：硬件初始化、主循环、日志记录
├── config.py                  # ⚙️ 全局参数（7 段 + MANEUVER 内部 A~K 子段）
├── calibration_hsv.json       # 🎨 赛场标定的 HSV 阈值（运行时热加载）
├── control/
│   ├── pid.py                 # 增量式 PID（微分先行，积分限幅）
│   └── motor.py               # GPIO/PWM + PID 速度环 + 差速/直通/倒车接口
├── sensors/
│   ├── camera.py              # 多线程捕获 + HSV 颜色识别 + 轮廓中心检测
│   ├── encoder.py             # 霍尔编码器 GPIO 中断计数
│   └── ks103.py               # I²C 超声波 + 中值滤波 + 面积融合校验
├── navigation/
│   ├── state_machine.py       # ⭐ 主状态机 v3.10（10 状态 + 超时保护）
│   └── maneuvers.py           # 黄/绿机动步骤定义
└── tools/
    ├── calibrate_hsv.py       # HSV 阈值实时校准工具
    └── cal_angle.py           # 编码器转角系数 K 标定工具
```

---

## 🧠 核心原理

### 1. 整体架构：传感器 → 状态机 → 执行器

```
┌───────────────┐     ┌─────────────────────┐     ┌──────────────────┐
│  传感器层      │────▶│    状态机层           │────▶│   执行器层         │
│               │     │                     │     │                  │
│ 摄像头(HSV)    │     │  SEARCH ──▶ 机动状态  │     │  PID 速度环(双轮)  │
│ 超声波(KS103)  │     │  MANEUVER            │     │  差速驱动(L298N)  │
│ 编码器(霍尔)   │     │  ALIGN_RED/APPROACH   │     │  直通模式(原地转)   │
│               │     │  TANGENT/ORBIT        │     │  倒车直通          │
│               │     │  POST_WAIT/SCAN_NEXT  │     │                  │
│               │     │  ALIGN_CUBE           │     │                  │
└───────────────┘     └─────────────────────┘     └──────────────────┘
```

**传感器层**多线程采集（摄像头独立线程、编码器 GPIO 中断、超声波 I²C 轮询），**状态机层**在每次 `tick()` 中读取快照、决策下发，**执行器层** PID 速度闭环独立线程运行。

### 2. 状态机总览 (v3.10)

```
黄/绿魔方：
  SEARCH（三阶段扫描，有超时保护）
   → MANEUVER（6+1 步序列）
     → POST_WAIT → SCAN_NEXT（有超时保护）→ ALIGN_CUBE → SEARCH / FINISH

红色魔方：
  SEARCH（三阶段扫描）
   → ALIGN_RED → APPROACH（距离P+倒车+防抖）→ TANGENT → ORBIT → RECOVER_RED
     → POST_WAIT → SCAN_NEXT → ALIGN_CUBE → SEARCH / FINISH
```

**SEARCH**：直行（超声+视觉）→ 左扫（纯视觉）→ 右扫（纯视觉），循环，每个扫描阶段有 6s 超时保底。解决魔方非直线排列遗漏问题。

**完成后流水线** (v3.1+)：POST_WAIT 停车消惯性 → SCAN_NEXT 原地旋转扫描 → ALIGN_CUBE PD 对准接近，大幅降低遗漏率。v3.10 增加超时保护。

### 3. 传感器融合：超声波为主，面积为辅

KS103 超声波提供绝对距离，但存在跳变和锥形波束误检。图像面积随距离有稳定**趋势性**，但难以精确标定。

融合逻辑（本组核心创新）：
```
新读数与中位值偏差 > 阈值时：
  ┌ 面积↑ + 超声骤减 → 锥形波束侧方反射   ✗ 丢弃
  ├ 面积↓ + 超声骤增 → 丢波/超量程         ✗ 丢弃
  └ 趋势一致          → 真实机动          ✓ 保守平滑
```

### 4. PID 控制：两层结构

#### 内层：电机速度 PID（MOTOR 段）— 完整 Kp/Ki/Kd

唯一的真正闭环 PID，编码器测速反馈，10Hz 运行，微分先行 + 积分限幅。

```
                        标定换算                          ┌─────┐
 占空比(0-100) ──▶ rated_speed × duty/base_speed ──▶ │ setpoint │
                                                     └──┬──┘
                    ┌─────┐   前馈(duty)     ┌────┐      │
                    │  +  │◀────────────────│ 电机 │◀────┤
                    └──┬──┘                 └────┘      │
                       △                                │
           ┌───────┐   │                                │
           │ PID   │◀──┘                                │
           │ (增量) │───────────────────────────────────┘
           └───────┘  反馈(编码器 → 实测转速)
```

前馈+反馈结构：目标占空比直接输出（响应快），PID 只修正偏差（精度高）。`rated_speed` 是标定常数，用户只调占空比，换算自动完成。

#### 外层：3 个精简控制器（刻意不用全 PID）

| 控制器 | 控制量 | P | I | D | 原因 |
|:---|:---|:---:|:---:|:---:|:---|
| `approach_kp_dist` | 距离→速度 | ✓ | | | 超声波噪声会使 I 积错、D 抖动；settle 防抖替代 I/D |
| `align_centering_kp` | 像素偏差→转向 | ✓ | | | 目标每帧变化，积旧偏差无意义 |
| `align_cube_kp` + `align_cube_kd` | 像素偏差→旋转速度 | ✓ | | ✓ | 原地旋转时图像稳定，D 抑振有效 |

#### 直通模式 (`set_raw`)

原地旋转和倒车时编码器不区分方向，绕过 PID 直接输出两轮占空比。角度判定：`(|dr| + |dl|) × K_rotate`。

### 5. 差速与倒车

```
# 前进差速（两轮正转，禁止倒转）
left  = duty × (1 + bias × turn_rate)     # 硬限幅 [min_duty, 100]
right = duty × (1 - bias × turn_rate)

# 原地旋转（set_raw 直通，一正一反）
左转：set_raw(-speed, +speed)   右转：set_raw(+speed, -speed)

# 倒车（set_target 负值 → _loop 直通输出负占空比）
set_target(-speed, 0)
```

### 6. 红色魔方：切向入轨绕行

```
ALIGN_RED(P对准前进) → APPROACH(距离P停车+倒车防抖) → TANGENT(原地右转切向)
→ ORBIT(差速弧线CCW绕行，编码器+视觉双重确认) → RECOVER_RED(回正)
```

**关键**：TANGENT/ORBIT 用编码器角度闭环判定，不依赖定时；ORBIT 完成后摄像头双重确认防累积误差。

### 7. 黄/绿魔方：原地旋转式通过 (v3)

```
approach(距离P)→stop(0.3s)→rotate1(原地转向)→straight(PID直行)→rotate2(原地回正)→straight(恢复+清离)
```

比旧版差速弧线优势：原地旋转精确可控，编码器角度判定，不依赖差速半径标定。

---

## 🚀 快速开始

```bash
# 1. 启动 pigpio 守护进程
sudo pigpiod

# 2. 运行主程序
sudo python3 main.py

# 3. 调试模式
sudo python3 main.py --debug           # 详细日志（每帧打印状态）
sudo python3 main.py --no-sonar        # 禁用超声波（纯视觉调试）
sudo python3 main.py --log data/       # 记录每帧数据到 CSV
sudo python3 main.py --freq 10         # 指定主循环频率（默认 15Hz）
```

---

## 🔧 关键配置速查

所有参数集中在 `config.py`（7 段 + MANEUVER 内部 A~K 子段），修改前请注释原因和日期。

### 一、摄像头 & 视觉

```python
CAMERA = {'id': 0, 'width': 320, 'height': 240, 'fps': 30, 'warmup_frames': 5}
VISION  = {'roi_y_start': 0.2, 'min_contour_area': 500, 'morph_kernel': 3}
# HSV 阈值 → 建议用 tools/calibrate_hsv.py 标定，自动保存到 calibration_hsv.json
```

### 二、电机 & 速度 PID

```python
MOTOR = {
    'base_speed':  40,       # 基准占空比（所有速度参数的参考点）
    'min_duty':     5,       # 最低占空比（电机不转时可提高到 8-10）
    'turn_rate':    0.6,     # 转向系数（越大转弯越紧）

    # 标定值（一次测定，不需调参）
    'rated_speed_l':  0.61,  # 左轮 @40%占空比 实测转速 (圈/0.1s)
    'rated_speed_r':  0.57,  # 右轮 @40%占空比 实测转速 (圈/0.1s)

    # PID 增益
    'pid_left':  {'Kp': 25, 'Ki': 0.01, 'Kd': 35},
    'pid_right': {'Kp': 25, 'Ki': 0.01, 'Kd': 40},
    'pid_period': 0.1,       # 采样周期 (秒)
}
```

### 三、MANEUVER 机动参数（config.py 第五段 A~K）

```python
MANEUVER = {
    # ── A. 标定系数（必须实车标定）──
    'deg_per_pulse_rotate': 0.14,   # 原地旋转 (度/脉冲之和)
    'deg_per_pulse_diff':   0.07,   # 弧线绕行 (度/脉冲差) ★★ 必须 cal_angle.py 标定
    'pulses_per_cm':        11.7,   # 直行距离 (脉冲/cm)

    # ── B. SEARCH 扫描 ──
    'search_straight_time': 3.0,    # 直行搜索时间 (秒)
    'scan_speed':           20,     # 扫描旋转占空比
    'scan_angle':           60,     # 单侧扫描角度 (度)
    'scan_phase_timeout':   6.0,    # 扫描超时防卡死 (秒)

    # ── C. 距离P控制（approach 步骤通用）──
    'stop_distance':          30,   # 黄/绿目标停止距离 (cm)
    'approach_kp_dist':       0.8,  # 距离P增益 (占空比/cm)
    'approach_min_speed':     10,   # 最低接近速度
    'approach_stop_tol':      3.0,  # 到达容差 (cm)
    'approach_settle_time':   0.3,  # 到位后防抖时间 (秒)
    'approach_settle_timeout':5.0,  # settle 总超时 (秒)

    # ── D/E. 黄/绿魔方 ──
    # 黄色：rotate1=左转45° → straight → rotate2=右转45°回正
    # 绿色：rotate1=右转45° → straight → rotate2=左转45°回正
    'yellow_rotate1_angle':  45, 'yellow_rotate1_speed': 30,
    'yellow_rotate2_angle':  45, 'yellow_rotate2_speed': 30,
    'yellow_pass_speed':     35, 'yellow_pass_dist':     50, 'yellow_pass_time': 1.5,
    'green_rotate1_angle':   45, 'green_rotate1_speed':  30,
    'green_rotate2_angle':   45, 'green_rotate2_speed':  30,
    'green_pass_speed':      35, 'green_pass_dist':      50, 'green_pass_time':  1.5,
    'recover_straight_dist': 20, 'post_maneuver_straight_time': 1.0,

    # ── F~J. 红色魔方 ──
    'align_centering_kp':    0.2,  # 对准P增益
    'align_red_speed':       26,   # 对准前进速度
    'orbit_radius':          45,   # 绕行半径 / 停车目标距离 (cm)
    'red_approach_settle_time': 0.5, 'red_approach_settle_timeout': 6.0,
    'tangent_target_deg':    15,   # 切向转角 (度)
    'tangent_speed':         26,
    'orbit_speed':           40,   'orbit_turn_bias': -0.19,
    'orbit_target_deg':      160,  'orbit_timeout': 25,
    'red_recover_speed':     35,   'red_recover_turn': 0.6, 'red_recover_time': 0.4,

    # ── K. 完成后扫描对准 (v3.1) ──
    'post_maneuver_wait':  0.5,    # 完成后等待 (秒)
    'scan_next_speed':     20,     'scan_next_angle': 50,  'scan_next_timeout': 6.0,
    'align_cube_kp':       0.25,   'align_cube_kd': 0.05,
    'align_cube_stop_dist': 35,    'align_cube_timeout': 8.0,
}
```

### 四、超声波

```python
ULTRASONIC = {'detect_dist': 70, 'action_dist': 30, 'clear_dist': 50}
KS103      = {'bus': 1, 'address': 0x74, 'filter_window': 5,
              'outlier_threshold_cm': 15, 'read_interval': 0.035}
```

---

## 📝 参数调优指南

| 现象 | 关键参数 |
|:---|:---|
| 电机不转/抖动 | `MOTOR['min_duty']` |
| 直行偏航（左右不对称）| `MOTOR['rated_speed_l/r']`（重新标定）|
| PID 振荡/响应慢 | `MOTOR['pid_left/right']['Kp/Kd']` |
| 转弯半径不对 | `MANEUVER['*_turn']` 或 `turn_rate` |
| 红色绕行角度不准 | `deg_per_pulse_diff`（必须 `cal_angle.py` 标定）|
| 原地旋转角度不准 | `deg_per_pulse_rotate`（重新标定）|
| 颜色识别失败 | 运行 `tools/calibrate_hsv.py` 重新标定 |
| 超声数据跳变 | `KS103['outlier_threshold_cm']`、`filter_window` |
| 动作超时/不够 | `MANEUVER['*_time']` 或 `*_timeout` |
| approach 停车不准/振荡 | `approach_kp_dist`、`approach_stop_tol`、`approach_settle_timeout` |
| approach 冲过头 | `approach_kp_dist`↓ 或 `approach_min_speed`↓ |
| 倒车不生效 | 检查 `min_duty` 是否太高、settle 超时是否太短 |
| 扫描电机不转/卡死 | `scan_speed`↑（≥20）、`scan_phase_timeout` |
| 扫描不到下一魔方 | `scan_next_speed`↑、`scan_next_angle`↑、`scan_next_timeout` |
| ALIGN_CUBE 对准振荡 | `align_cube_kp`↓ 或 `align_cube_kd`↑ |

---

## 🐛 已知问题与修复历程

| 问题 | 版本 | 修复 |
|:---|:---|:---|
| 差速过大致轮子倒转 | v2 | 比例差速 + 硬限幅 `[min_duty, 100]` |
| PID setpoint 指数漂移 | v2 | 用固定 `rated_speed` 换算，不用被污染的 setpoint |
| 左右电机不对称 | v2 | 差异化 rated_speed + 差异化 Kd |
| 红色定时硬编码不可靠 | v3 | 编码器角度累计 + 超声距离融合 |
| 状态机单 tick 跳过步骤 | v3 | `_advance_step()` 每次只推进一步 |
| KS103 跳变/锥形波束误检 | v3 | 中值滤波 + 面积趋势校验 |
| 制动方向与前进相同 | v3 | 左轮制动引脚方向修正 |
| 完成后遗漏下一魔方 | v3.1 | POST_WAIT → SCAN_NEXT → ALIGN_CUBE |
| 红 APPROACH 定速冲过头 | v3.9 | 距离P停车 + 超调倒车 + settle 防抖 |
| 倒车不生效 | v3.10 | settle 退出时重置总超时；红 APPROACH 进入时清零 settle 变量 |
| 原地扫描电机堵转卡死 | v3.10 | 扫描速度 15→20；扫描阶段加超时保护 |
| 扫描阶段切换方向惯性 | v3.10 | 阶段切换前先 `set_target(0,0)` 停车 |
| 参数体系混乱 | v3.10 | config 重组为 7 段 + A~K 子段；速度统一为占空比 |

---

## ⚠️ 重要提醒

1. **先启动 pigpiod**：`sudo pigpiod`（否则 KS103 初始化失败）
2. **root 权限运行**：`sudo python3 main.py`（GPIO 权限要求）
3. **Ctrl+C 安全退出**：信号处理已封装，自动释放硬件
4. **赛场前必须重新标定**：HSV 阈值 + 编码器转角系数 (`cal_angle.py`)
5. **`deg_per_pulse_diff` 默认不准**：必须 `cal_angle.py` 实车标定
6. **KS103 VCC 串联 100Ω 电阻**：防过压损坏
7. **`rated_speed` 是标定常数**：一次测定后不改，所有速度参数统一用占空比

---

> **维护**：JackeryTU | **最后更新**：2026-06-11  |  **版本**：v3.10
