"""
control/pid.py
增量式 PID 控制器（纯算法，无硬件依赖）

设计要点：
• 微分先行（Derivative on Measurement）：对 setpoint 跳变免疫，
  适合 PIDSpeedLoop 每周期动态调整目标速度的工况。
• 积分限幅防止 windup。
"""


class PIDController:
    """
    单轮增量式 PID。

    参数
    ----
    Kp, Ki, Kd : PID 增益
    ideal_speed : 标称目标速度（圈/采样周期），作为 setpoint 基准

    注意：Ki 已隐含固定采样周期 dt（config.py 中的 pid_period），
          若将来改采样周期，需同步调整 Ki。
    """

    def __init__(self, Kp: float, Ki: float, Kd: float, ideal_speed: float):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = ideal_speed

        self._prev_measured: float = 0.0
        self._integral: float = 0.0
        self._integral_limit = 50.0

    def reset(self):
        self._prev_measured = 0.0
        self._integral = 0.0

    def compute(self, measured_speed: float) -> float:
        """
        输入本采样周期的实测速度，返回占空比调整量。
        调用方负责将结果叠加到前馈上并钳制到 [0, 100]。
        """
        error = self.setpoint - measured_speed
        self._integral += error
        self._integral = max(-self._integral_limit,
                             min(self._integral_limit, self._integral))

        # 微分先行：只对测量值变化敏感，对 setpoint 跳变免疫
        derivative = -(measured_speed - self._prev_measured)
        self._prev_measured = measured_speed

        return self.Kp * error + self.Ki * self._integral + self.Kd * derivative