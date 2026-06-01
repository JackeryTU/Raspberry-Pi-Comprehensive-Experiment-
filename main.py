"""
main.py
魔方绕桩小车 —— 主程序入口

负责：
  1. 初始化所有硬件（电机、编码器、摄像头、超声波）
  2. 启动状态机主循环（10~20 Hz）
  3. 安全退出与资源清理
  4. 可选：命令行参数控制调试模式、日志记录

用法：
    sudo python3 main.py
    sudo python3 main.py --debug          # 开启详细日志
    sudo python3 main.py --no-sonar       # 禁用超声波（纯视觉调试）
    sudo python3 main.py --log data/      # 记录每帧数据到CSV

注意：
    • 必须先启动 pigpiod：sudo pigpiod
    • 必须以 root 权限运行（GPIO 权限要求）
    • 按 Ctrl+C 安全退出
"""

import os
import sys
import time
import signal
import argparse
import csv
from datetime import datetime

# 将项目根目录加入路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import MOTOR, ULTRASONIC, CAMERA
from control.motor import init as motor_init, cleanup as motor_cleanup
from sensors.camera import Camera, detect_cube
from sensors.ks103 import KS103
from sensors.encoder import get_encoder_counts
from navigation.state_machine import CubeStateMachine

# ==================== 全局运行标志 ====================
_running = True


def _signal_handler(signum, frame):
    """捕获 SIGINT / SIGTERM，实现优雅退出。"""
    global _running
    print("\n[main] 收到终止信号，准备安全退出...")
    _running = False


signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)


# ==================== 日志记录器（可选） ====================

class DataLogger:
    """
    每帧记录关键数据到 CSV，便于赛后复盘与参数调优。
    字段：时间戳、状态、超声波距离、目标颜色、编码器(右,左)、FPS
    """

    def __init__(self, out_dir: str):
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(out_dir, f"run_{ts}.csv")
        self.file = open(self.path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "timestamp", "state", "target_color", "dist_cm",
            "enc_r", "enc_l", "cam_fps", "notes"
        ])
        print(f"[logger] 数据记录开启: {self.path}")

    def log(self, state_name: str, color: str, dist, enc_r: int, enc_l: int,
            fps: int, notes: str = ""):
        self.writer.writerow([
            f"{time.time():.3f}", state_name, color or "None",
            f"{dist:.1f}" if dist is not None else "None",
            enc_r, enc_l, fps, notes
        ])

    def close(self):
        self.file.close()
        print("[logger] 数据记录已关闭")


# ==================== 主函数 ====================

def main():
    parser = argparse.ArgumentParser(description="魔方绕桩小车主程序")
    parser.add_argument("--debug", action="store_true",
                        help="开启详细调试日志（每帧打印状态机内部信息）")
    parser.add_argument("--no-sonar", action="store_true",
                        help="禁用 KS103 超声波，仅用摄像头调试（状态机中距离相关逻辑会失效）")
    parser.add_argument("--log", type=str, default=None,
                        help="指定日志输出目录，记录每帧数据到 CSV")
    parser.add_argument("--freq", type=float, default=15.0,
                        help="主循环频率(Hz)，默认15")
    args = parser.parse_args()

    # ---------- 硬件初始化 ----------
    print("=" * 50)
    print("  魔方绕桩小车 —— 主程序启动")
    print("=" * 50)

    print("[main] 初始化电机与编码器...")
    motor_init()

    print("[main] 启动摄像头...")
    cam = Camera().start()
    time.sleep(0.5)  # 等待摄像头线程稳定

    sonar = None
    if not args.no_sonar:
        print("[main] 初始化 KS103 超声波...")
        try:
            sonar = KS103()
        except RuntimeError as e:
            print(f"[main] 警告：KS103 初始化失败 ({e})")
            print("[main] 将以纯视觉模式运行（部分状态机逻辑可能异常）")
    else:
        print("[main] 已禁用超声波（--no-sonar）")

    # ---------- 状态机初始化 ----------
    print("[main] 初始化状态机...")
    fsm = CubeStateMachine(camera=cam, sonar=sonar)
    fsm.start()

    # ---------- 日志初始化 ----------
    logger = None
    if args.log:
        logger = DataLogger(args.log)

    # ---------- 主循环 ----------
    period = 1.0 / args.freq
    tick_count = 0
    last_status_time = time.monotonic()

    print(f"[main] 主循环启动，频率 {args.freq} Hz（周期 {period*1000:.1f} ms）")
    print("[main] 按 Ctrl+C 退出\n")

    try:
        while _running:
            t0 = time.monotonic()

            # ---- 获取传感器数据 ----
            frame = cam.get_frame()
            dist = sonar.get_distance() if sonar else None

            # 若需要超声波-面积融合校验，可在此提取面积传给 sonar
            # （当前 state_machine.tick() 内部已自行调用 sonar.get_distance()，
            #  此处获取仅用于日志记录和调试显示）
            area = None
            if frame is not None and fsm.target_color:
                _, area = detect_cube(frame, fsm.target_color)
                # 融合校验：将面积传给超声波驱动
                if sonar and area is not None:
                    dist = sonar.get_distance(area=area)

            # ---- 状态机推进 ----
            active = fsm.tick()
            if not active:
                print("[main] 状态机报告任务完成，退出主循环")
                break

            # ---- 日志记录 ----
            if logger and tick_count % 5 == 0:  # 每5帧记录一次，降低IO
                enc_r, enc_l = get_encoder_counts()
                logger.log(
                    state_name=fsm.state.name,
                    color=fsm.target_color,
                    dist=dist,
                    enc_r=enc_r,
                    enc_l=enc_l,
                    fps=cam.get_fps(),
                    notes=f"step_idx={fsm._step_idx}"
                )

            # ---- 定时打印状态（降低刷屏） ----
            if args.debug or (time.monotonic() - last_status_time >= 1.0):
                dist_s = f"{dist:.1f}" if dist is not None else "None"
                area_s = f"{int(area)}" if area else "None"
                print(f"[main] state={fsm.state.name:12s} color={fsm.target_color or '---':6s} "
                      f"dist={dist_s:>6}cm area={area_s:>6} fps={cam.get_fps():2d}")
                last_status_time = time.monotonic()

            tick_count += 1

            # ---- 定频休眠 ----
            elapsed = time.monotonic() - t0
            sleep_t = period - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)
            elif args.debug:
                print(f"[main] 警告：主循环超时 {abs(sleep_t)*1000:.1f} ms，"
                      f"实际频率低于设定值")

    except Exception as e:
        print(f"\n[main] 主循环异常: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # ---------- 安全清理 ----------
        print("\n[main] 正在安全清理资源...")
        fsm.stop()
        cam.stop()
        if sonar:
            sonar.close()
        motor_cleanup()
        if logger:
            logger.close()
        print("[main] 所有资源已释放，程序退出")


if __name__ == "__main__":
    main()
