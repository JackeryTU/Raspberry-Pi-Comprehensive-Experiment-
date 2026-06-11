---
name: project-overview
description: 魔方绕桩小车 v3.10 项目结构、硬件、设计约束
metadata:
  type: project
---

# 魔方绕桩小车 v3.10

## 硬件
- 树莓派 + L298N 电机驱动 + 霍尔编码器（仅计数正向脉冲，无方向）
- KS103 I2C 超声波 + USB 摄像头 320×240
- 两轮差速，I1/I4 高电平前进，I2/I3 高电平反转

## 任务
按黄→绿→红顺序绕过 3 个魔方，到达终点。黄色左侧通过，绿色右侧通过，红色 360° 逆时针绕行。

## 关键约束
- 霍尔编码器 **不区分方向**，始终累加计数
- 原地旋转角度用 `|dr| + |dl| × K_rotate` 估算
- PID 速度环独立线程运行（周期 0.1s），有 `set_raw()` 直通模式用于原地旋转
- 倒车走 `_loop()` 负 base 分支（非 passthrough 模式），直通输出负占空比
- 扫描阶段依赖编码器角度判定退出，无编码器脉冲则永久卡死 → v3.10 加了超时保护

## 文件结构
- `config.py` — 全部可调参数（含扫描超时保护参数）
- `control/motor.py` — GPIO、PWM、PID 速度环、set_raw 直通、倒车直通
- `control/pid.py` — 增量式 PID（微分先行、积分限幅）
- `navigation/state_machine.py` — 主状态机 v3.10（含扫描超时保护、阶段切换停车、settle变量清零）
- `navigation/maneuvers.py` — 黄/绿机动步骤定义
- `sensors/camera.py` — 摄像头 + HSV 颜色识别
- `sensors/encoder.py` — 霍尔编码器脉冲计数（GPIO中断）
- `sensors/ks103.py` — KS103 超声波 + 面积融合校验
- `main.py` — 主循环入口（10~20Hz）

## v3.10 关键修复
- 倒车不生效：settle退出时重置 `_settle_total_start`，红APPROACH进入时清零settle变量
- 原地旋转不转：扫描速度 15→20，扫描阶段加超时保护防堵转卡死
- 阶段切换方向惯性：`_start_search_phase()` 和 SCAN_NEXT 切换前先 `set_target(0,0)` 停车

**Why:** 了解项目整体架构后再做修改，避免破坏已知设计约束。
**How to apply:** 修改前先读此文件和相关源码，注意编码器不区分方向的影响。
