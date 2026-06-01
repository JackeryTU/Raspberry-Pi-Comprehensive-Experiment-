#!/usr/bin/env python3
"""
tools/calibrate_hsv.py
HSV 颜色阈值实时校准工具（赛场专用）

用法：
    python3 tools/calibrate_hsv.py

操作说明：
    r / y / g       切换当前校准颜色（red / yellow / green）
    1 / 2           红色专用：切换编辑区间1或区间2（红色跨HSV首尾）
    s               保存当前所有颜色阈值到 calibration_hsv.json
    l               重新加载 calibration_hsv.json（覆盖当前）
    q / ESC         退出

特性：
    • 切换颜色时自动保存当前滑条值，防止误操作丢失
    • 实时显示原图 + 掩码 + 检测框，直观验证效果
    • 保存的 JSON 可被 camera.py / config.py 自动加载
    • 兼容 OpenCV 3.x / 4.x，已禁用所有运行时警告
"""

# ========== 警告抑制：必须在 import cv2 之前 ==========
import os
os.environ['OPENCV_LOG_LEVEL'] = 'ERROR'
os.environ['GST_DEBUG'] = '0'
import warnings
warnings.filterwarnings('ignore')
# ======================================================

import cv2
import numpy as np
import json
import sys

# 将项目根目录加入路径以导入 config
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import config


CALIBRATION_FILE = config.CALIBRATION_FILE


def nothing(_):
    """OpenCV trackbar 回调占位函数"""
    pass


def load_thresholds():
    """从 JSON 文件加载阈值，失败则返回 config 默认值"""
    if os.path.exists(CALIBRATION_FILE):
        try:
            with open(CALIBRATION_FILE, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            print(f"[!] 加载校准文件失败: {e}")
    return config.COLOR_THRESHOLDS


def save_thresholds(thresholds):
    """保存阈值到 JSON 文件"""
    try:
        with open(CALIBRATION_FILE, 'w', encoding='utf-8') as f:
            json.dump(thresholds, f, indent=2, ensure_ascii=False)
        print(f"[+] 阈值已保存到 {CALIBRATION_FILE}")
    except Exception as e:
        print(f"[!] 保存失败: {e}")


def get_trackbar_values():
    """读取 6 个滑条当前值"""
    return [
        cv2.getTrackbarPos('H_low',  'HSV_Calibrator'),
        cv2.getTrackbarPos('S_low',  'HSV_Calibrator'),
        cv2.getTrackbarPos('V_low',  'HSV_Calibrator'),
    ], [
        cv2.getTrackbarPos('H_high', 'HSV_Calibrator'),
        cv2.getTrackbarPos('S_high', 'HSV_Calibrator'),
        cv2.getTrackbarPos('V_high', 'HSV_Calibrator'),
    ]


def set_trackbar_values(low, high):
    """设置 6 个滑条位置"""
    cv2.setTrackbarPos('H_low',  'HSV_Calibrator', low[0])
    cv2.setTrackbarPos('S_low',  'HSV_Calibrator', low[1])
    cv2.setTrackbarPos('V_low',  'HSV_Calibrator', low[2])
    cv2.setTrackbarPos('H_high', 'HSV_Calibrator', high[0])
    cv2.setTrackbarPos('S_high', 'HSV_Calibrator', high[1])
    cv2.setTrackbarPos('V_high', 'HSV_Calibrator', high[2])


def apply_current_to_memory(thresholds, color, red_idx, low, high):
    """将当前滑条值写入内存中的 thresholds 字典"""
    if color == 'red':
        # 确保 red 有两个区间槽位
        while len(thresholds['red']['ranges']) < 2:
            thresholds['red']['ranges'].append([[0, 0, 0], [179, 255, 255]])
        thresholds['red']['ranges'][red_idx] = [low, high]
    else:
        thresholds[color]['ranges'] = [[low, high]]


def main():
    # ---------- 初始化 ----------
    thresholds = load_thresholds()

    # 当前状态
    current_color = 'green'
    red_range_idx = 0   # 红色当前编辑的区间（0 或 1）

    # 启动摄像头
    cap = cv2.VideoCapture(config.CAMERA['id'])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA['width'])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA['height'])

    # 创建窗口与滑条
    cv2.namedWindow('HSV_Calibrator', cv2.WINDOW_NORMAL)
    cv2.createTrackbar('H_low',  'HSV_Calibrator', 0,   179, nothing)
    cv2.createTrackbar('H_high', 'HSV_Calibrator', 179, 179, nothing)
    cv2.createTrackbar('S_low',  'HSV_Calibrator', 0,   255, nothing)
    cv2.createTrackbar('S_high', 'HSV_Calibrator', 255, 255, nothing)
    cv2.createTrackbar('V_low',  'HSV_Calibrator', 0,   255, nothing)
    cv2.createTrackbar('V_high', 'HSV_Calibrator', 255, 255, nothing)

    def refresh_trackbars():
        """根据 current_color + red_range_idx 刷新滑条"""
        if current_color == 'red':
            ranges = thresholds['red']['ranges']
            idx = red_range_idx if red_range_idx < len(ranges) else 0
            low, high = ranges[idx]
        else:
            low, high = thresholds[current_color]['ranges'][0]
        set_trackbar_values(low, high)

    refresh_trackbars()

    print("=" * 50)
    print("  HSV 颜色阈值校准工具")
    print("=" * 50)
    print("  r / y / g     切换颜色 (red / yellow / green)")
    print("  1 / 2         红色：切换编辑区间1 / 区间2")
    print("  s             保存所有阈值到 calibration_hsv.json")
    print("  l             从 calibration_hsv.json 重新加载")
    print("  q / ESC       退出")
    print("=" * 50)
    print(f"[*] 初始阈值来源: {'calibration_hsv.json' if os.path.exists(CALIBRATION_FILE) else 'config.py 默认值'}")

    # ---------- 主循环 ----------
    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 读取当前滑条值
        low, high = get_trackbar_values()

        # 生成预览掩码
        if current_color == 'red':
            # 红色：合并内存中已保存的两个区间 + 当前滑条实时预览
            mask = cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8))
            for i, (rl, rh) in enumerate(thresholds['red']['ranges']):
                if i != red_range_idx:
                    m = cv2.inRange(hsv, np.array(rl, dtype=np.uint8), np.array(rh, dtype=np.uint8))
                    mask = cv2.bitwise_or(mask, m)
        else:
            mask = cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8))

        # 形态学去噪（与正式代码一致）
        k = config.VISION['morph_kernel']
        if k > 1:
            kernel = np.ones((k, k), np.uint8)
            mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        else:
            mask_clean = mask

        # 找轮廓并画框
        result = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = result[0] if len(result) == 2 else result[1]

        vis = frame.copy()
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area >= config.VISION['min_contour_area']:
                x, y, bw, bh = cv2.boundingRect(largest)
                cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                cv2.putText(vis, f"{current_color} {int(area)}",
                           (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # 信息叠加
        info_lines = [
            f"Color: {current_color}" + (f" range{red_range_idx + 1}" if current_color == 'red' else ""),
            f"H:[{low[0]:3d},{high[0]:3d}] S:[{low[1]:3d},{high[1]:3d}] V:[{low[2]:3d},{high[2]:3d}]",
            "r/y/g=切换  1/2=红区间  s=保存  l=加载  q=退出",
        ]
        for i, txt in enumerate(info_lines):
            cv2.putText(vis, txt, (5, 15 + i * 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # 拼接原图与掩码显示
        mask_bgr = cv2.cvtColor(mask_clean, cv2.COLOR_GRAY2BGR)
        display = np.hstack((vis, mask_bgr))
        cv2.imshow('HSV_Calibrator', display)

        # ---------- 键盘响应 ----------
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:
            break

        elif key == ord('r'):
            apply_current_to_memory(thresholds, current_color, red_range_idx, low, high)
            current_color = 'red'
            red_range_idx = 0
            refresh_trackbars()
            print("[*] 切换至: 红色 区间1")

        elif key == ord('y'):
            apply_current_to_memory(thresholds, current_color, red_range_idx, low, high)
            current_color = 'yellow'
            refresh_trackbars()
            print("[*] 切换至: 黄色")

        elif key == ord('g'):
            apply_current_to_memory(thresholds, current_color, red_range_idx, low, high)
            current_color = 'green'
            refresh_trackbars()
            print("[*] 切换至: 绿色")

        elif key == ord('1') and current_color == 'red':
            apply_current_to_memory(thresholds, current_color, red_range_idx, low, high)
            red_range_idx = 0
            refresh_trackbars()
            print("[*] 红色 -> 区间1")

        elif key == ord('2') and current_color == 'red':
            apply_current_to_memory(thresholds, current_color, red_range_idx, low, high)
            red_range_idx = 1
            refresh_trackbars()
            print("[*] 红色 -> 区间2")

        elif key == ord('s'):
            # 先保存当前滑条值，再写入文件
            apply_current_to_memory(thresholds, current_color, red_range_idx, low, high)
            save_thresholds(thresholds)

        elif key == ord('l'):
            thresholds = load_thresholds()
            refresh_trackbars()
            print("[+] 已重新加载校准文件")

    # ---------- 清理 ----------
    cap.release()
    cv2.destroyAllWindows()
    print("[*] 校准工具已退出")


if __name__ == '__main__':
    import time
    main()
