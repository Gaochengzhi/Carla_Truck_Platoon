import casadi as ca
import carla
import numpy as np
from typing import List, Tuple


class VehicleModel:
    def __init__(self, vehicle: carla.Vehicle):
        self.vehicle = vehicle
        self.wheelbase = 2.9  # 轴距(m)
        self.max_steer_angle = np.deg2rad(45.0)  # 最大转向角(rad)
        self.max_throttle = 1.0  # 最大油门
        self.max_brake = 1.0  # 最大刹车
        self.max_acceleration = 1.0  # 最大加速度(m/s^2)
        self.max_deceleration = -1.0  # 最大减速度(m/s^2)
        self.lf = 1.5  # 前轮到车辆质心的距离(m)
        self.lr = 1.4  # 后轮到车辆质心的距离(m)
        self.nx = 6  # 状态向量维度
        self.nu = 2  # 控制输入维度
        self.previous_u = np.zeros(self.nu)  # 上一时刻的控制输入
        self.dt = 0.1  # 时间步长(s)

    def update_state(self, dt: float) -> Tuple[float, float, float]:
        """
        更新车辆状态
        :param dt: 时间间隔(s)
        :return: (x, y, yaw) 车辆位置和航向角
        """
        # 获取当前车辆状态
        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()

        # 车辆当前位置和航向角
        x = transform.location.x
        y = transform.location.y
        yaw = np.deg2rad(transform.rotation.yaw)

        # 车辆当前速度大小和方向
        speed = np.linalg.norm([velocity.x, velocity.y, velocity.z])
        direction = np.array([np.cos(yaw), np.sin(yaw)])

        # 更新车辆位置
        x += speed * direction[0] * dt
        y += speed * direction[1] * dt

        return x, y, yaw

    def dynamics(self, x: ca.SX, u: ca.SX) -> ca.SX:
        """
        Vehicle dynamics model
        :param x: Vehicle state vector (x, y, yaw, v, a, delta)
        :param u: Control input vector (a, delta)
        :return: Next state vector
        """
        # Extract states and inputs
        x_pos = x[0]
        y_pos = x[1]
        yaw = x[2]
        v = x[3]
        a = u[0]
        delta = u[1]

        # Update states based on the vehicle dynamics equations
        x_next = x_pos + v * ca.cos(yaw) * self.dt
        y_next = y_pos + v * ca.sin(yaw) * self.dt
        yaw_next = yaw + (v / self.wheelbase) * ca.tan(delta) * self.dt
        v_next = v + a * self.dt
        a_next = a
        delta_next = delta

        # Return the next state vector
        return ca.vertcat(x_next, y_next, yaw_next, v_next, a_next, delta_next)

        # Return the next state vector
        return ca.vertcat(x_next, y_next, yaw_next, v_next, a_next, delta_next)

    def apply_control(self, throttle: float, steer: float, brake: float):
        """
        应用控制指令
        :param throttle: 油门(0~1)
        :param steer: 转向角(-1~1)
        :param brake: 刹车(0~1)
        """
        # 转向角转换
        steer_angle = steer * self.max_steer_angle

        # 油门和刹车转换
        throttle_control = np.clip(throttle, 0, self.max_throttle)
        brake_control = np.clip(brake, 0, self.max_brake)

        # 应用控制指令
        self.vehicle.apply_control(carla.VehicleControl(
            throttle=throttle_control,
            steer=steer_angle,
            brake=brake_control
        ))

    def objective_function(self, x: ca.SX, u: ca.SX, target_speed: float, target_waypoints: List[Tuple[float, float]]) -> ca.SX:
        """
        目标函数
        :param x: 车辆状态向量 (x, y, yaw, v)
        :param u: 控制输入向量 (a, delta)
        :param target_speed: 目标速度
        :param target_waypoints: 目标路点列表,每个路点为(x, y)元组
        :return: 目标函数值
        """
        # 状态追踪误差权重
        Q_x = 1.0  # 横向位置误差权重
        Q_y = 1.0  # 纵向位置误差权重
        Q_yaw = 1.0  # 航向角误差权重
        Q_v = 0.5  # 速度误差权重

        # 控制输入变化率权重
        R_a = 0.1  # 加速度变化率权重
        R_delta = 0.1  # 转向角变化率权重

        # 舒适性约束权重
        Q_acc = 1.0  # 加速度约束权重
        Q_jerk = 1.0  # 加加速度约束权重

        # 计算状态追踪误差
        x_error = x[0] - target_waypoints[0][0]
        y_error = x[1] - target_waypoints[0][1]

        # 计算航向角误差
        if len(target_waypoints) > 1:
            yaw_error = x[2] - ca.atan2(target_waypoints[1][1] - target_waypoints[0][1],
                                        target_waypoints[1][0] - target_waypoints[0][0])
        else:
            yaw_error = x[2] - x[2]  # 如果只有一个目标点，航向角误差为0

        v_error = x[3] - target_speed

        # 计算控制输入变化率
        a_diff = u[0] - self.previous_u[0]
        delta_diff = u[1] - self.previous_u[1]

        # 计算加速度和加加速度
        acc = u[0]
        jerk = (u[0] - self.previous_u[0]) / self.dt

        # 目标函数值
        cost = Q_x * x_error**2 + Q_y * y_error**2 + Q_yaw * yaw_error**2 + Q_v * v_error**2 + \
            R_a * a_diff**2 + R_delta * delta_diff**2 + \
            Q_acc * acc**2 + Q_jerk * jerk**2

        return cost

    def velocity_constraints(self, x: ca.SX, v_min: float, v_max: float) -> List[ca.SX]:
        v = x[3]  # 当前速度
        v_lower = ca.fmax(v_min, v - self.max_deceleration * self.dt)
        v_upper = ca.fmin(v_max, v + self.max_acceleration * self.dt)
        return [v_lower <= v, v <= v_upper]

    def acceleration_constraints(self, u_min: float, u_max: float) -> List[ca.SX]:
        a = ca.SX.sym('a')
        return [u_min <= a, a <= u_max]

    def steering_constraints(self, u_min: float, u_max: float) -> List[ca.SX]:
        delta = ca.SX.sym('delta')
        return [u_min <= delta, delta <= u_max]

    def lateral_acceleration_constraints(self, x: ca.SX, u: ca.SX, ay_max: float) -> List[ca.SX]:
        v = x[3]  # 当前速度
        delta = u[1]  # 转向角
        ay = v**2 * ca.tan(delta) / self.wheelbase  # 横向加速度
        return [-ay_max <= ay, ay <= ay_max]

    def tire_slip_angle_constraints(self, x: ca.SX, u: ca.SX, alpha_max: float) -> List[ca.SX]:
        v = x[3]  # 当前速度
        delta = u[1]  # 转向角
        alpha_f = ca.atan2(v * ca.sin(delta), v *
                           ca.cos(delta) + self.lf * delta) - delta  # 前轮侧偏角
        alpha_r = ca.atan2(v * ca.sin(delta), v *
                           ca.cos(delta) - self.lr * delta)  # 后轮侧偏角
        return [-alpha_max <= alpha_f, alpha_f <= alpha_max, -alpha_max <= alpha_r, alpha_r <= alpha_max]


class MPCController:
    def __init__(self, model, N=10, dt=0.03):
        self.model = model
        self.N = N  # 预测时域步数
        self.dt = dt  # 时间步长

    def solve(self, x0, target_speed, target_waypoints):
        """
        求解MPC问题
        :param x0: 初始状态向量
        :param target_speed: 目标速度
        :param target_waypoints: 目标路点列表
        :return: 最优控制序列的第一个元素
        """
        # 定义状态变量
        x = ca.SX.sym('x', self.model.nx, self.N + 1)

        # 定义控制输入变量
        a = ca.SX.sym('a', self.N)
        delta = ca.SX.sym('delta', self.N)
        u = ca.vertcat(a, delta)

        # 定义目标函数
        cost = 0
        for k in range(self.N):
            cost += self.model.objective_function(
                x[:, k], u[k::self.model.nu], target_speed, target_waypoints)

        # 定义约束条件
        constraints = []
        for k in range(self.N):
            # 动力学约束
            constraints += [x[:, k + 1] ==
                            self.model.dynamics(x[:, k], u[k::self.model.nu])]

            # 状态约束
            constraints += self.model.velocity_constraints(x[:, k], 0, 20)

            # 控制输入约束
            constraints += self.model.acceleration_constraints(u[k], -1, 1)
            constraints += self.model.steering_constraints(
                u[k + self.N], -np.pi / 4, np.pi / 4)

            # 横向加速度约束
            constraints += self.model.lateral_acceleration_constraints(
                x[:, k], u[k::self.model.nu], 5)

            # 轮胎侧偏角约束
            constraints += self.model.tire_slip_angle_constraints(
                x[:, k], u[k::self.model.nu], np.deg2rad(5))

        # 初始状态约束
        constraints += [x[:, 0] == x0]

        # 定义优化问题
        nlp = {'x': ca.vertcat(ca.reshape(x, -1, 1), u),
               'f': cost,
               'g': ca.vertcat(*constraints)}

        # 设置优化选项
        opts = {'ipopt.print_level': 0, 'print_time': 0}

        # 创建求解器
        solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

        # 设置初始猜测值
        x_init = np.zeros((self.model.nx, self.N + 1))
        u_init = np.zeros((self.model.nu * self.N, 1))
        x_init[:, 0] = x0
        lbx = np.zeros((self.model.nx * (self.N + 1) +
                       self.model.nu * self.N, 1))
        ubx = np.zeros((self.model.nx * (self.N + 1) +
                       self.model.nu * self.N, 1))
        lbx[0:self.model.nx * (self.N + 1):self.model.nx] = x0
        ubx[0:self.model.nx * (self.N + 1):self.model.nx] = x0

        # 求解优化问题
        sol = solver(x0=ca.vertcat(ca.reshape(x_init, -1, 1), u_init),
                     lbx=lbx, ubx=ubx, lbg=0, ubg=0)

        # 提取最优控制序列的第一个元素
        u_opt = sol['x'][-self.model.nu *
                         self.N:].reshape((self.model.nu, self.N))

        return u_opt[:, 0]
