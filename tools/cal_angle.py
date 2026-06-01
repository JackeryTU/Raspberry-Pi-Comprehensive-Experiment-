# 临时测试：绕魔方一圈
from control.motor import init, PIDSpeedLoop, cleanup
from sensors.encoder import reset_encoder_counts, get_encoder_counts
import time

init()
loop = PIDSpeedLoop()
loop.start()

# 用和正式比赛一样的参数绕魔方
loop.set_target(speed=25, turn_bias=0.5)  # 正式用的 orbit_arc_turn

input(">>> 按回车开始绕魔方...")
reset_encoder_counts()

# 让小车绕，你肉眼观察，回到魔方右侧时按 Ctrl+C
try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    pass

r, l = get_encoder_counts()
print(f"脉冲差: {r - l}")
K = 360.0 / abs(r - l)
print(f"K = {K:.6f}")

loop.stop()
cleanup()