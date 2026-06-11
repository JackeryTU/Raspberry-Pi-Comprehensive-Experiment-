"""
sensors/camera.py
摄像头驱动 + HSV颜色识别

设计要点（适配老旧树莓派）：
1. 多线程独立捕获，避免图像IO阻塞主控制循环
2. 低分辨率 320x240 + ROI裁剪，减少计算量
3. 单颜色指定检测（抗干扰），同时提供多颜色扫描接口
4. 轮廓法求中心（避免遍历全图或计算图像矩）
5. 兼容 OpenCV 3.x / 4.x 的 findContours 返回值差异
6. 颜色阈值支持从外部 calibration_hsv.json 热加载（赛场校准）
7. 彻底禁用 OpenCV / GStreamer 运行时警告
"""

# ========== 警告抑制：必须在任何 import 之前 ==========
import os
import sys
import ctypes

# ---- 终极方案：重定向 C 级别的 stderr 到 /dev/null ----
# libjpeg 直接写文件描述符 2，绕过 Python 的 sys.stderr
libc = ctypes.CDLL(None)
devnull = os.open('/dev/null', os.O_WRONLY)
libc.dup2(devnull, 2)  # 将 fd 2 (stderr) 重定向到 /dev/null
os.close(devnull)
# 注意：此后所有 C 库的 stderr 输出（包括 libjpeg）都会被丢弃
# -----------------------------------------------------

os.environ['OPENCV_LOG_LEVEL'] = 'ERROR'
os.environ['GST_DEBUG'] = '0'

import warnings
warnings.filterwarnings('ignore')
# ======================================================

import cv2
import numpy as np
import threading
import time
import sys

# 将项目根目录加入路径以导入 config
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import config


class Camera:
    """摄像头后台捕获类"""

    def __init__(self, cam_id=None):
        self.cam_id = cam_id if cam_id is not None else config.CAMERA['id']
        self.cap = None
        self.frame = None          # 最新帧（由后台线程更新）
        self._work = False         # 线程运行标志
        self._thread = None
        self._lock = threading.Lock()

        # FPS统计
        self._fps_counter = 0
        self._fps = 0
        self._last_fps_time = 0

    def start(self):
        """启动摄像头和后台捕获线程"""
        self.cap = cv2.VideoCapture(self.cam_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开摄像头设备 /dev/video{self.cam_id}")

        # 设置低分辨率以减轻老旧树莓派负担
        # 若设置失败（部分驱动不支持），不影响运行，只是帧率可能略低
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA['width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA['height'])
        self.cap.set(cv2.CAP_PROP_FPS, config.CAMERA['fps'])

        # 丢弃前N帧，等待自动曝光和白平衡稳定
        for _ in range(config.CAMERA['warmup_frames']):
            self.cap.read()

        self._work = True
        self._last_fps_time = time.time()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        return self

    def _capture_loop(self):
        """后台线程：持续读取最新帧"""
        while self._work:
            ret, frame = self.cap.read()
            if ret:
                with self._lock:
                    self.frame = frame
                self._fps_counter += 1
            else:
                # 读取失败时短暂休眠，避免CPU空转
                time.sleep(0.01)

            # 每秒更新一次FPS统计
            now = time.time()
            if now - self._last_fps_time >= 1.0:
                with self._lock:
                    self._fps = self._fps_counter
                self._fps_counter = 0
                self._last_fps_time = now

    def get_frame(self):
        """获取当前最新帧（非阻塞，可能返回None）"""
        with self._lock:
            return self.frame

    def get_fps(self):
        """获取最近1秒的实际平均帧率"""
        with self._lock:
            return self._fps

    def stop(self):
        """安全关闭摄像头"""
        self._work = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()
            self.cap = None

    def __del__(self):
        self.stop()


def get_mask(hsv, color_name):
    """
    根据颜色名生成HSV二值掩码。
    阈值自动从 config.get_color_thresholds() 获取（支持赛场热加载）。

    参数:
        hsv: HSV色彩空间的numpy数组
        color_name: 'red', 'yellow', 'green'
    返回:
        mask: 二值图像（numpy uint8），目标颜色区域为255
    """
    thresholds = config.get_color_thresholds()
    color_cfg = thresholds.get(color_name)
    if not color_cfg:
        raise ValueError(f"未知颜色: {color_name}")

    mask = None
    for low, high in color_cfg['ranges']:
        lo = np.array(low, dtype=np.uint8)
        up = np.array(high, dtype=np.uint8)
        m = cv2.inRange(hsv, lo, up)
        if mask is None:
            mask = m
        else:
            # 红色需要合并两个区间
            mask = cv2.bitwise_or(mask, m)
    return mask


def detect_cube(frame, target_color):
    """
    在图像中检测指定颜色的魔方。
    采用"指定颜色检测"策略，抗干扰能力强，计算量小。

    参数:
        frame: BGR图像（numpy数组）
        target_color: 'red', 'yellow', 'green'
    返回:
        (center_x, area):
            center_x — 魔方中心在图像中的x坐标（像素）；未找到返回None
            area     — 轮廓面积；可用于粗略估算距离；未找到返回None
    """
    if frame is None:
        return None, None

    h, w = frame.shape[:2]

    # ROI裁剪：只处理图像下半部分，减少计算量并过滤非地面干扰
    roi_y = int(h * config.VISION['roi_y_start'])
    roi = frame[roi_y:, :]

    # 转为HSV色彩空间
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # 获取颜色掩码（自动加载最新校准）
    mask = get_mask(hsv, target_color)

    # 轻量形态学开运算：去除3x3以下的噪点
    k = config.VISION['morph_kernel']
    if k > 1:
        kernel = np.ones((k, k), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    # 兼容 OpenCV 3.x（返回3个值）和 4.x（返回2个值）
    result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(result) == 3:
        _, contours, _ = result
    else:
        contours, _ = result

    if not contours:
        return None, None

    # 取面积最大的轮廓（假设为魔方）
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < config.VISION['min_contour_area']:
        return None, None

    # 计算轮廓中心x坐标：将轮廓点展平后用numpy求均值
    # 比 Python for 循环快，比 cv2.moments 计算量小
    pts = largest.reshape(-1, 2)          # shape: (N, 2)
    center_x_roi = int(np.mean(pts[:, 0]))

    # 映射回原图坐标系（ROI裁剪不影响x坐标，只影响y；此处只需x）
    center_x = center_x_roi

    return center_x, area


def identify_nearest_cube(frame):
    """
    识别当前画面中面积最大的魔方，并判断其颜色。
    适用于超声波发现障碍物后，近距离确认魔方颜色。

    注意：此函数需连续检测3种颜色，计算量为单颜色的3倍，
          建议在减速/停车状态下调用，不要在高速搜索阶段每帧调用。

    返回:
        (color_name, center_x, area):
            均未找到时返回 (None, None, None)
    """
    best = None  # (color, cx, area)
    for color in ['red', 'yellow', 'green']:
        cx, area = detect_cube(frame, color)
        if area is not None and (best is None or area > best[2]):
            best = (color, cx, area)

    if best is None:
        return None, None, None
    return best


def judge_position(center_x, frame_width):
    """
    根据魔方中心x坐标判断其位于画面左/中/右。
    后续控制逻辑可据此决定小车转向方向。

    参数:
        center_x: 像素坐标
        frame_width: 图像宽度
    返回:
        'left', 'center', 'right', 或 'lost'（未找到）
    """
    if center_x is None:
        return 'lost'
    third = frame_width // 3
    if center_x < third:
        return 'left'
    elif center_x > 2 * third:
        return 'right'
    else:
        return 'center'


# ==================== 本地测试入口 ====================
if __name__ == '__main__':
    """
    独立运行此文件可进行摄像头颜色识别可视化调试：

    在树莓派终端执行：
        python3 sensors/camera.py

    按 'q' 退出，按 'r'/'y'/'g' 切换测试颜色。
    若已运行 tools/calibrate_hsv.py 生成 calibration_hsv.json，
    本程序会自动加载赛场校准后的阈值。
    """
    print("[*] 启动摄像头...")
    cam = Camera().start()
    time.sleep(2)  # 等待线程稳定

    target = 'green'  # 默认测试颜色
    print(f"[*] 当前测试颜色: {target} (按 r/y/g 切换, q 退出)")
    print(f"[*] 阈值来源: {'calibration_hsv.json' if os.path.exists(config.CALIBRATION_FILE) else 'config.py 默认值'}")

    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            h, w = frame.shape[:2]
            cx, area = detect_cube(frame, target)
            pos = judge_position(cx, w)

            # 可视化（调试用；正式运行时请注释掉 imshow 以节省CPU）
            vis = frame.copy()
            if cx is not None:
                # 画中心点（y坐标画在画面中间方便观察）
                cv2.circle(vis, (cx, h // 2), 6, (0, 255, 0), -1)
                cv2.putText(vis, f"{target.upper()}: {pos}  area={int(area)}  FPS={cam.get_fps()}",
                           (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(vis, f"{target.upper()}: not found  FPS={cam.get_fps()}",
                           (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow("Cube Detect", vis)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                target = 'red'
                print("[+] 切换至红色")
            elif key == ord('y'):
                target = 'yellow'
                print("[+] 切换至黄色")
            elif key == ord('g'):
                target = 'green'
                print("[+] 切换至绿色")

            # 终端打印（降低频率避免刷屏）
            print(f"\r[FPS:{cam.get_fps():2d}] {target}: pos={pos}, cx={cx}, area={area}", end="")
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        cv2.destroyAllWindows()
        print("\n[*] 摄像头已安全关闭")
