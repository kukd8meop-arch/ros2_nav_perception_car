"""Ivmech PID Controller is simple implementation of a Proportional-Integral-Derivative (PID) Controller in the Python Programming Language.
More information about PID Controller: http://en.wikipedia.org/wiki/PID_controller
"""
import time

class PID:
    """PID Controller
    """

    def __init__(self, P=0.2, I=0.0, D=0.0):

        self.Kp = P # 比例
        self.Ki = I # 积分
        self.Kd = D # 微分

        # 最小计算周期（秒）
        # 如果 delta_time < sample_time，则本次 update 不计算
        self.sample_time = 0.00
        # 当前时间（秒）
        self.current_time = time.time()
        # 上一次计算的时间，用于计算 Δt
        self.last_time = self.current_time
        # 初始化/清空 PID 内部状态
        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        # 目标值 r(t)
        self.SetPoint = 0.0

        # PID三项的中间计算结果
        self.PTerm = 0.0 # 比例项：Kp * e(t)，e(t)是误差
        self.ITerm = 0.0 # 积分累加项：∫e(t)dt（尚未乘 Ki）
        self.DTerm = 0.0 # 微分项：de(t)/dt（尚未乘 Kd）
        self.last_error = 0.0 # 上一次的误差 e(t-1)，用于计算微分项

        # 抗积分饱和（windup）相关
        self.int_error = 0.0 # （历史遗留变量，这里实际上没用）
        self.windup_guard = 20.0 # 积分项的最大绝对值限制

        self.output = 0.0 # PID 最终输出 u(t)

    def update(self, feedback_value):
        """
        根据当前反馈值 feedback_value 计算 PID 输出

        数学形式：
        u(t) = Kp * e(t)
             + Ki * ∫ e(t) dt
             + Kd * de(t)/dt
        """
        # 当前误差 e(t) = 目标值 - 反馈值
        error = self.SetPoint - feedback_value 

        self.current_time = time.time()  # 获取当前时间
        delta_time = self.current_time - self.last_time # 计算时间差 Δt
        delta_error = error - self.last_error # 计算误差变化量 Δe

        # 如果已经达到设定的采样周期，才进行 PID 计算
        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error # 比例项 PTerm = Kp * e(t)
            self.ITerm += error * delta_time # 积分项 ITerm = ∫ e(t) dt ≈ Σ e(t) * Δt

            # 对积分项进行限幅，防止积分饱和（windup）
            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            # 微分项
            self.DTerm = 0.0 # 初始化为 0
            if delta_time > 0:
                # DTerm = de(t)/dt ≈ (e(t) - e(t-1)) / Δt
                self.DTerm = delta_error / delta_time 

            # 保存本次时间和误差，供下一次 update 使用
            self.last_time = self.current_time
            self.last_error = error

            # PID输出 u(t) = PTerm + Ki * ITerm + Kd * DTerm
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        # 设置比例系数 Kp
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        # 设置积分系数 Ki
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        # 设置微分系数 Kd
        self.Kd = derivative_gain

    def setWindup(self, windup):
        # 设置积分项最大绝对值，用于防止积分饱和
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        # 设置 PID 的最小计算周期（秒）
        self.sample_time = sample_time


if __name__ == '__main__':
    x_pid = PID(P=0.2, I=0, D=0)
    x_pid.SetPoint = 5
    x_pid.update(10)
    out = x_pid.output
    print(out)
