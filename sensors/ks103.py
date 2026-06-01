"""
sensors/ks103.py
KS103 超声波测距驱动 + 超声/面积融合策略

设计要点（适配老旧树莓派 + 吸取外组教训）：
1. 统一使用 pigpio 进行 I2C 通信（与电机控制库一致，减少依赖）
2. 中值滤波 + 异常值剔除，解决 KS103 数据跳变问题
3. 与图像面积融合校验：面积趋势作为超声波读数的"合理性判据"
   —— 这是本组对"面积处理"的核心看法，与外组"完全放弃超声"不同
4. 供电安全：代码注释中明确提醒 VCC 串联 100Ω 电阻

关于"面积"的独立看法（与外组方案的区别）：
------------------------------------------
外组因 KS103 跳变和锥形波束问题，最终完全放弃超声波，改用图像面积估算距离。
但我们认为：
  • 面积-距离关系并非固定映射，它受魔方朝向、摄像头俯仰角、部分遮挡、
    光照变化影响极大，难以在赛场上精确标定；
  • 超声波提供的是绝对距离，虽然存在跳变和锥形误检，但在"非异常"状态下
    精度远高于面积反推；
  • 因此最佳策略是"超声波为主、面积为辅"：面积不直接换算成距离，
    而是作为超声波读数的"趋势校验器"。

融合逻辑：
  当超声波新读数与历史 median 偏差超过阈值时，对比图像面积变化趋势：
    - 面积在增大（远离）但超声显示骤减 → 超声检测到地面/侧方反射（锥形波束），丢弃
    - 面积在减小（靠近）但超声显示骤增 → 超声丢波，丢弃
    - 趋势一致 → 接受读数，可能只是小车快速机动导致
  这样既保留了超声波的绝对精度，又利用面积的趋势稳定性弥补了其跳变缺陷。
"""

import pigpio
import time
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import config


class KS103:
    """KS103 超声波测距驱动（带滤波与面积融合）"""

    def __init__(self, bus=None, address=None, mode=None):
        # 从 config 读取默认参数，允许运行时覆盖
        self.bus = bus if bus is not None else config.KS103['bus']
        self.address = address if address is not None else config.KS103['address']
        self.mode = mode if mode is not None else config.KS103['mode']

        # 连接 pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError(
                "pigpio 守护进程未启动。请在终端执行: sudo pigpiod\n"
                "若未安装: sudo apt-get install pigpio"
            )

        # 打开 I2C 设备
        self.device = self.pi.i2c_open(self.bus, self.address)

        # 滤波历史缓冲区
        self._history = []
        self._max_history = config.KS103['filter_window']

        # 面积-距离联合记录（用于趋势校验）
        self._last_area = None
        self._last_dist = None

    def _read_raw(self):
        """
        向 KS103 发送测距指令并读取原始距离（单位：cm）。
        若通信失败返回 None。
        """
        try:
            # 向寄存器 0x02 写入测距模式指令
            self.pi.i2c_write_byte_data(self.device, 0x02, self.mode)
            # KS103 最大测距时间约 33ms，留足余量 35ms
            time.sleep(config.KS103['read_interval'])
            # 读取高字节（0x02）和低字节（0x03）
            high = self.pi.i2c_read_byte_data(self.device, 0x02)
            low = self.pi.i2c_read_byte_data(self.device, 0x03)
            dist = ((high << 8) + low) / 10.0
            return dist
        except Exception as e:
            # I2C 通信异常时返回 None，由上层使用历史值兜底
            return None

    def get_distance(self, area=None):
        """
        获取滤波后的距离，支持用图像面积进行异常值校验。

        参数:
            area: 当前帧中魔方的轮廓面积（由 camera.detect_cube 返回）。
                  若提供，则启用面积-超声融合校验；若 None，则仅做纯超声滤波。
        返回:
            float: 滤波后的距离（cm）；缓冲区为空且通信失败时返回 None
        """
        raw = self._read_raw()

        if raw is None:
            # 通信失败：返回历史滤波值（如果有）
            return self._median_filter()

        # ---------- 异常值检测与面积融合校验 ----------
        if self._history:
            median = self._median_filter()
            deviation = abs(raw - median)

            if deviation > config.KS103['outlier_threshold_cm']:
                # 读数跳变，进入面积趋势校验
                if area is not None and self._last_area is not None and self._last_dist is not None:
                    # 计算面积变化趋势（带 20% 死区，避免噪声误触发）
                    area_increasing = area > self._last_area * 1.2
                    area_decreasing = area < self._last_area * 0.8
                    dist_increasing = raw > self._last_dist * 1.2
                    dist_decreasing = raw < self._last_dist * 0.8

                    # 矛盾场景 1：面积增大（远离）但超声骤减 → 地面/侧方反射（锥形波束）
                    if area_increasing and dist_decreasing:
                        # 丢弃本次异常读数，返回历史滤波值
                        return self._median_filter()

                    # 矛盾场景 2：面积减小（靠近）但超声骤增 → 丢波/超量程
                    if area_decreasing and dist_increasing:
                        return self._median_filter()

                    # 趋势一致：可能是真实快速机动，接受读数但做保守平滑
                    raw = median + (raw - median) * 0.5
                else:
                    # 无面积辅助时，对异常值做保守平滑（向 median 拉回）
                    raw = median + (raw - median) * 0.3

        # ---------- 更新历史缓冲区 ----------
        self._history.append(raw)
        if len(self._history) > self._max_history:
            self._history.pop(0)

        # 更新联合记录
        if area is not None:
            self._last_area = area
        self._last_dist = raw

        return self._median_filter()

    def _median_filter(self):
        """中值滤波：对脉冲型跳变数据抑制效果优于滑动平均"""
        if not self._history:
            return None
        sorted_hist = sorted(self._history)
        return sorted_hist[len(sorted_hist) // 2]

    def get_raw_history(self):
        """调试用：返回原始历史读数列表"""
        return list(self._history)

    def reset_filter(self):
        """重置滤波缓冲区（如切换魔方或重新搜索时调用）"""
        self._history.clear()
        self._last_area = None
        self._last_dist = None

    def close(self):
        """安全关闭 I2C 连接"""
        try:
            self.pi.i2c_close(self.device)
            self.pi.stop()
        except Exception:
            pass

    def __del__(self):
        self.close()


# ==================== 本地测试入口 ====================
if __name__ == '__main__':
    """
    独立运行此文件可测试 KS103 通信与滤波效果：

    在树莓派终端执行：
        sudo pigpiod          # 先启动守护进程（若未运行）
        python3 sensors/ks103.py

    按 Ctrl+C 退出。
    测试时可手持魔方前后移动，观察读数是否稳定；
    若出现跳变，可调整 config.py 中的 outlier_threshold_cm。
    """
    print("[*] 初始化 KS103...")
    print("[!] 提醒：请确认 KS103 VCC 已串联 100Ω 电阻再接 3.3V")

    try:
        sonar = KS103()
    except RuntimeError as e:
        print(e)
        sys.exit(1)

    print("[*] 开始测距（每 50ms 一次，Ctrl+C 退出）...")
    print("-" * 40)

    try:
        while True:
            dist = sonar.get_distance(area=None)  # 纯超声模式测试
            hist = sonar.get_raw_history()
            print(f"  滤波距离: {dist:6.1f} cm  |  原始历史: {[f'{d:.1f}' for d in hist]}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        sonar.close()
        print("\n[*] KS103 已安全关闭")
