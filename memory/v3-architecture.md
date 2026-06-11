---
name: v3-architecture
description: v3 架构决策——SEARCH 扫描、MANEUVER 统一状态、原地旋转、参数独立化、v3.10 修复
metadata:
  type: project
---

# v3 架构决策

## SEARCH 三阶段扫描

**问题**：魔方非直线排列，直行搜索容易遗漏。

**方案**：SEARCH 分三个子阶段循环，扫描阶段用纯摄像头检测（不依赖超声波）：

1. `straight` — 直行 3s，超声波+摄像头双重确认
2. `scan_left` — 原地慢速左转 60°，纯摄像头
3. `scan_right` — 原地慢速右转 120°（双倍覆盖+回中），纯摄像头

任一阶段摄像头发现魔方 → 立即锁定颜色，进入对应机动。

**参数**：`search_straight_time`, `scan_speed`(20), `scan_angle`, `scan_phase_timeout`(6s)

## MANEUVER 统一状态（替代 ALIGN / PASS / RECOVER）

**问题**：旧架构ALIGN→PASS→RECOVER用差速弧线转弯，RECOVER 时 PID setpoint 指数漂移导致疯狂顺时针旋转。

**方案**：单个 MANEUVER 状态执行 6+1 步序列，每步按 `action` 分发：

1. `approach` — 距离PID直行接近到 `stop_distance`(30cm)，含超调倒车+settle防抖
2. `stop` — 暂停 0.3s 消除惯性
3. `rotate` — 原地旋转（一正一反），编码器脉冲之和判定角度
4. `straight` — PID 直行通过
5. `rotate` — 原地旋转回正
6. `straight` — 短距恢复
7. `straight` — 直行清离魔方区域（纯时间判定）

**Why:** 原地旋转比差速弧线更精确可控，步骤化设计便于独立调参。
**How to apply:** 黄/绿魔方检测后 `_load_maneuver(color)` → `_enter(MANEUVER)` 即可。

## PID setpoint 指数漂移修复

**问题**：`control/motor.py` 中 `_loop()` 使用 `self._pid.setpoint`（已被修改的值）作为下次计算的 base，导致 setpoint 每轮迭代乘以 factor×ratio，指数级漂移。

**修复**：存储固定 `_rated_speed_l/r`（来自 config 的 `rated_speed` 标定值），每次循环用固定值计算 setpoint：`setpoint = rated_speed × factor × (duty / base_speed)`。

## 原地旋转实现

- `_set_left/right` 支持负 duty（反转）：duty>0 前进，duty<0 反转
- `PIDSpeedLoop.set_raw(l, r)` 直通模式，绕过 PID 直接设两轮 duty
- 角度判定：`angle = (|dr| + |dl|) × K_rotate`，因为编码器不区分方向

## 参数独立化

每个旋转步骤和黄/绿颜色均有独立参数：
- `yellow_rotate1_angle/speed`, `yellow_rotate2_angle/speed`
- `green_rotate1_angle/speed`, `green_rotate2_angle/speed`
- `yellow_pass_speed/time`, `green_pass_speed/time`
- `approach_speed_yg` 黄绿专用接近速度（独立于红色的 `approach_speed`）

**Why:** 第一次旋转（接近转向）和第二次旋转（回正）可能需要不同角度；黄/绿通过方向不同需要独立调参。
**How to apply:** 在 `config.py` 中直接修改对应参数，无需改代码。

## v3.1 完成后扫描对准（POST_WAIT → SCAN_NEXT → ALIGN_CUBE）

**问题**：v3 完成魔方后直接跳转 SEARCH，小车朝向可能偏离下一魔方，SEARCH 需要较长时间才能重新发现目标。

**方案**：在每个魔方后插入三段流水线：
1. POST_WAIT — 停车等待消除惯性
2. SCAN_NEXT — 原地旋转（左→右），纯摄像头扫描下一个魔方
3. ALIGN_CUBE — PD 摄像头比例对准 + 前进接近到 `align_cube_stop_dist`

之后进入 SEARCH，由于魔方已在近距离且居中，SEARCH 可立即检测并路由。

详细参数见 [[v3.1-post-maneuver]]。

## v3.10 关键修复

### 倒车不生效（Bug #1~3）

**问题**：距离PID的 settle 阶段因惯性冲过头时，退出 settle 下达倒车指令，但 `_settle_total_start` 未重置。该计时器从进入 approach 就开始跑，settle 期间已消耗数秒，下一个 tick 可能直接触发总超时→`_advance_step()`→跳过倒车。

**修复**：
1. `_enter(State.APPROACH)` 清零 `_settle_start` / `_settle_total_start`（防止残留值污染）
2. 黄/绿 MANEUVER settle 退出时重置 `_settle_total_start = time.monotonic()`
3. 红 APPROACH settle 退出时同上

### 原地扫描不转/卡死（Bug #4~8）

**问题 A**：SEARCH scan_left/right 和 SCAN_NEXT left/right 阶段**无超时保护**，仅靠编码器角度判定退出。若 `scan_speed=15` 太低导致电机堵转，编码器脉冲永不增长，小车永久卡死。

**修复**：所有扫描阶段添加超时检测（`scan_phase_timeout=6s`, `scan_next_timeout=6s`），超时强制推进到下一阶段。

**问题 B**：`_start_search_phase()` 和 SCAN_NEXT 阶段切换时未停车，上一阶段的 `set_raw` 指令继续跑一个 tick（~67ms），导致切换瞬间朝旧方向旋转。

**修复**：阶段切换前先 `self.loop.set_target(0, 0)` 停车。

**问题 C**：`scan_speed = 15` 对于原地旋转（一正一反需克服双倍静摩擦）偏临界。

**修复**：提高到 20。

**Why:** 扫描是纯编码器判定，无超时则堵转=死锁；切换不停车则方向惯性残留。
**How to apply:** 如需调扫描速度，修改 `config.py` 中 `scan_speed` / `scan_next_speed`（建议 ≥20）。
