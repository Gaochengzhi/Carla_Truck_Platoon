import numpy as np
import math

class CACC_Model:
    def __init__(self, t_0=0.5,s_0 = 2):
        pass
    def calc_speed(self,leader_v,
                leader_a,
                front_dis,
                front_v,
                front_a,
                ego_v,
                ego_a, leader_delay=0, front_delay=0, leader_tv = 0):
        pass




class IDM():
    def __init__(self, speed_limit=30, max_acc=5, normal_decel=5, delta=4, safe_distance=5.5, human_reaction_time=0.0, max_brake_decel=6.0):
        self.v0 = speed_limit
        self.delta = delta
        self.a = max_acc
        self.b = normal_decel
        self.max_brake_decel = max_brake_decel
        self.s0 = safe_distance
        self.T = human_reaction_time
        self.sqrtab = np.sqrt(max_acc*normal_decel)

    def calc_speed(self,leader_v,
                leader_a,
                front_dis,
                front_v,
                front_a,
                ego_v,
                ego_a, leader_delay=0, front_delay=0, leader_tv = 0):
        self.s0 = max(ego_v*0.5+2, 3)
        if front_dis < 4:
            return -self.max_brake_decel
        delta_v = ego_v-front_v
        s_star_raw = self.s0 + ego_v * self.T\
            + (ego_v * delta_v) / (2 * self.sqrtab)
        s_star = max(s_star_raw, self.s0)
        acc = self.a * (1 - np.power(ego_v / self.v0,
                        self.delta) - (s_star ** 2) / (front_dis**2))
        acc = max(acc, -self.max_brake_decel)
        return acc+ego_v

class SUMO_CACC_Model:
    def __init__(self,t_0=0.5, s_0=5):
        self.t_0 = t_0
        self.s_0 = s_0
        self.xi = 1.0
        self.omega_n = 0.2
        self.c1 = 0.5
        self.alpha1 = 1 - self.c1
        self.alpha2 = self.c1
        self.alpha3 = -(2 * self.xi - self.c1 * (self.xi + (self.xi**2 - 1)**0.5)) * self.omega_n
        self.alpha4 = -(self.xi + (self.xi**2 - 1)**0.5) * self.omega_n * self.c1
        self.alpha5 = -self.omega_n**2

    def calc_speed(self, leader_v, leader_a, front_dis, front_v, front_a, ego_v, ego_a,
                   leader_delay=0, front_delay=0, leader_tv=0):
        spacing_error = -(front_dis - self.t_0 * ego_v + 2)
        speed_error = ego_v - front_v

        cacc_a = (self.alpha1 * front_a +
                  self.alpha2 * leader_a +
                  self.alpha3 * speed_error +
                  self.alpha4 * (ego_v - leader_v) +
                  self.alpha5 * spacing_error)

        return ego_v + cacc_a


class PloegModel:
    def __init__(self, h=0.5, k_p=0.2, k_d=0.7):
        self.h = h
        self.k_p = k_p
        self.k_d = k_d

    def calc_speed(self, leader_v, leader_a, front_dis, front_v, front_a, ego_v, ego_a,
                   leader_delay=0, front_delay=0, leader_tv=0):
        ploeg_a = (1 / self.h * (
            -ego_a +
            self.k_p * (front_dis - (2 + self.h * ego_v)) +
            self.k_d * (front_v - ego_v - self.h * ego_a) +
            front_a
        ))

        return ego_v + ploeg_a

class PFL_CACC:
    def __init__(self, c_p=4.5, c_v=0.23, c_a=0.1, k_v=0.5, k_a=0.2, s_0=0.5):
        """
        Initialize the PLF controller.

        :param c_p: spacing error gain between the ego vehicle and the preceding vehicle
        :param c_v: velocity error gain between the ego vehicle and the preceding vehicle
        :param c_a: acceleration error gain between the ego vehicle and the preceding vehicle
        :param k_v: velocity error gain between the ego vehicle and the leading vehicle
        :param k_a: acceleration error gain between the ego vehicle and the leading vehicle
        :param s_0: desired time headway
        """
        self.c_p = c_p
        self.c_v = c_v
        self.c_a = c_a
        self.k_v = k_v
        self.k_a = k_a
        self.s_0 = s_0

    def calc_speed(self,leader_v,
                leader_a,
                front_dis,
                front_v,
                front_a,
                ego_v,
                ego_a, leader_delay=0, front_delay=0, leader_tv = 0):
        spacing_error = front_dis - self.s_0 * ego_v-2
        velocity_error_front = front_v - ego_v
        acceleration_error_front = front_a - ego_a
        velocity_error_leader = leader_v - ego_v
        acceleration_error_leader = leader_a - ego_a

        u = self.c_p * spacing_error + self.c_v * velocity_error_front + self.c_a * acceleration_error_front \
            + self.k_v * velocity_error_leader + self.k_a * acceleration_error_leader
        return u

class Path_CACC:
    def __init__(self, kp=0.45, kd=0.25, s0=2, tc=0.5):
        """
        Initialize the PATH CACC controller.

        :param kp: The proportional gain for the spacing error.
        :param kd: The derivative gain for the spacing error.
        :param s0: The desired minimum safety spacing.
        :param tc: The desired time headway.
        """
        self.kp = kp
        self.kd = kd
        self.s0 = s0
        self.headway = tc

    def calc_speed(self, leader_v, leader_a, front_dis, front_v, front_a, ego_v, ego_a, leader_delay=0, front_delay=0, leader_tv=0):
            """
            Calculate the speed using the CACC controller.

            :param leader_v: Speed of the leading vehicle.
            :param leader_a: Acceleration of the leading vehicle.
            :param front_dis: Front vehicle distance.
            :param front_v: Speed of the front vehicle.
            :param front_a: Acceleration of the front vehicle.
            :param ego_v: Ego vehicle speed.
            :param ego_a: Ego vehicle acceleration.
            :param leader_delay: Delay of the leader vehicle (default is 0).
            :param front_delay: Delay of the front vehicle (default is 0).
            :param leader_tv: Leader vehicle time gap (default is 0).
            """
            # Calculate the desired spacing
            des_spacing = self.s0 + self.headway * ego_v 
            
            # Calculate the spacing error
            spacing_error = front_dis - des_spacing

            # Calculate the control input
            control_input = self.kp * spacing_error + self.kd * (front_v - ego_v)

            # Update the ego vehicle speed
            new_speed = ego_v + control_input

            return new_speed


class Path_ACC:
    def __init__(self, k1=0.53, k2=2, s_0=0.5):
        """
        Initialize the ACC controller.
        :param k1: spacing error gain
        :param k2: relative velocity gain
        :param T: desired time gap
        """
        self.k1 = k1
        self.k2 = k2
        self.s_0 = s_0

    def calc_speed(self, target_v, ego_v, back_dis):
        """
        Calculate the control input for the ACC controller.
        :param vi_1: velocity of the preceding vehicle
        :param vi: velocity of the ego vehicle
        :param xi_1: position of the preceding vehicle
        :param xi: position of the ego vehicle
        :return: control input ai
        """
        max_acc = [-1.5,1.5]
        delat_t = 0.034

        # e = back_dis - self.s_0 * ego_v-2
        # e = back_dis - 5
        # speed = ego_v+ self.k1 * e + self.k2 * (target_v - ego_v)

        return target_v
        # return 20


class SUMO_CACC_Controllerback:
    def __init__(self, headway_time=0.5, speed_control_gain=-0.4, gap_control_gain_gap=0.45,
                 gap_control_gain_gap_dot=0.0125, collision_avoidance_gain_gap=0.45,
                 collision_avoidance_gain_gap_dot=0.05, emergency_threshold=2.0, headway_time_acc=1.0,
                 speed_control_min_gap=1.66):
        self.headway_time = headway_time
        self.speed_control_gain = speed_control_gain
        self.gap_control_gain_gap = gap_control_gain_gap
        self.gap_control_gain_gap_dot = gap_control_gain_gap_dot
        self.collision_avoidance_gain_gap = collision_avoidance_gain_gap
        self.collision_avoidance_gain_gap_dot = collision_avoidance_gain_gap_dot
        self.emergency_threshold = emergency_threshold
        self.headway_time_acc = headway_time_acc
        self.speed_control_min_gap = speed_control_min_gap

    def calc_speed(self,leader_v,
                leader_a,
                front_dis,
                front_v,
                front_a,
                ego_v,
                ego_a, leader_delay=0, front_delay=0, leader_tv = 0):
        leading_v_err = ego_v - leader_v
        time_gap = front_dis / max(ego_v, 0.1)
        spacing_err = front_dis - self.headway_time * ego_v - self.emergency_threshold
        speed_err = front_v - ego_v + self.headway_time * ego_a
        if abs(time_gap) > self.headway_time+0.2 or abs(spacing_err) > self.speed_control_min_gap:
            return ego_v + self.gap_control_gain_gap * spacing_err + self.gap_control_gain_gap_dot * speed_err
        elif spacing_err < -self.speed_control_min_gap:
            return ego_v + self.collision_avoidance_gain_gap * spacing_err + self.collision_avoidance_gain_gap_dot * speed_err
        else:
            return ego_v + self.speed_control_gain * leading_v_err 



class SUMO_CACC_Controller:
    def __init__(self, headway_time=0.5, speed_control_gain=-0.4, gap_control_gain_gap=0.45,
                 gap_control_gain_gap_dot=0.0125, collision_avoidance_gain_gap=0.45,
                 collision_avoidance_gain_gap_dot=0.05, emergency_threshold=2.0, headway_time_acc=1.0,
                 speed_control_min_gap=1.66):
        self.headway_time = headway_time
        self.speed_control_gain = speed_control_gain
        self.gap_control_gain_gap = gap_control_gain_gap
        self.gap_control_gain_gap_dot = gap_control_gain_gap_dot
        self.collision_avoidance_gain_gap = collision_avoidance_gain_gap
        self.collision_avoidance_gain_gap_dot = collision_avoidance_gain_gap_dot
        self.emergency_threshold = emergency_threshold
        self.headway_time_acc = headway_time_acc
        self.speed_control_min_gap = speed_control_min_gap

    def calc_speed(self, leader_v, leader_a, front_dis, front_v, front_a, ego_v, ego_a, leader_delay=0, front_delay=0, leader_tv=0):
        # Calculate the desired spacing
        des_spacing = self.headway_time * ego_v + self.emergency_threshold
        spacing_err = front_dis - des_spacing
        speed_err = front_v - ego_v
        v_err = ego_v - leader_v
        
        if abs(spacing_err) < 0.5:
            # speed mode
            return ego_v + self.speed_control_gain * v_err + self.gap_control_gain_gap_dot*spacing_err
            

        elif spacing_err <= -15:
            # Collision avoidance mode
            new_speed = ego_v + self.collision_avoidance_gain_gap * spacing_err + \
                        self.collision_avoidance_gain_gap_dot * v_err
            return max(0, new_speed)
        else:
            # CACC closing mode 
            return ego_v + self.gap_control_gain_gap * spacing_err + 0.2* speed_err



class Plexe_CACC_Controller:
    def __init__(self, kp=1.0, lambda_=0.1, spacing=5.0, xi=1.0, omega_n=0.2, c1=0.5):
        """
        Initialize the CACC controller.

        :param kp: Proportional gain for Cruise Control.
        :param lambda_: Time gap for Adaptive Cruise Control.
        :param spacing: Desired spacing in Cooperative Adaptive Cruise Control.
        :param xi: Damping ratio for CACC.
        :param omega_n: Natural frequency for CACC.
        :param c1: Constant defining contribution of acceleration in CACC.
        """
        self.kp = kp
        self.lambda_ = lambda_
        self.spacing = spacing
        self.xi = xi
        self.omega_n = omega_n
        self.c1 = c1
        # Calculating additional constants for the CACC control law
        self.alpha1 = 1 - c1
        self.alpha2 = c1
        self.alpha3 = -(2 * xi - c1 * (xi + (xi**2 - 1)**0.5)) * omega_n
        self.alpha4 = -omega_n**2

    def calc_acceleration(self, ego_v, lead_v, lead_a, front_v, front_a, front_dist):
        """
        Calculate the desired acceleration for the ego vehicle using CACC.

        :param ego_v: Ego vehicle speed.
        :param ego_a: Ego vehicle acceleration.
        :param lead_v: Leading vehicle speed.
        :param lead_a: Leading vehicle acceleration.
        :param front_v: Front vehicle speed.
        :param front_a: Front vehicle acceleration.
        :param front_dist: Distance to front vehicle.
        :return: Desired acceleration.
        """
        # CACC target following error
        epsilon = front_dist - self.spacing
        # Speed difference to the front vehicle
        epsilon_dot = ego_v - front_v
        # CACC acceleration
        acc_cacc = (self.alpha1 * front_a + self.alpha2 * lead_a + self.alpha3 * epsilon_dot + 
                    self.alpha4 * (ego_v - lead_v) + self.alpha4 * epsilon)
        return acc_cacc


import numpy as np

class HInfinityController:
    def __init__(self):
        self.h = 0.5  # 时距
        self.k1 = 0.15  # 对应论文中的k_i
        self.epsilon = 1  # 论文中没有给出具体值,这里假设为1
        self.delta = 0.5  # 对应论文中的gamma
        self.q = 0.8  # 耦合滑模参数,对应论文中的beta
        self.ppp = 0.3  # 对应论文中的alfa_iS
        self.lanmda10 = 10
        self.lanmda11 = 8
        self.lanmda20 = 5
        self.lanmda21 = 6
        self.lip1 = 0.5
        self.lip2 = 1.5
        self.am = -3  # 最小加速度
        self.aM = 3  # 最大加速度
        self.Tstep = 0.03  # 时间步长

        self.dd1_hat = 0  # 位置扰动估计初值
        self.dd1_hat_dot = 0  # 位置扰动变化率估计初值
        self.dd2_hat = 0  # 速度扰动估计初值
        self.dd2_hat_dot = 0  # 速度扰动变化率估计初值
        self.intOfUr = 0  # ur的积分初值

    def calc_speed(
        self,
        leader_v,
        leader_a,
        front_dis,
        front_v,
        front_a,
        ego_v,
        ego_a,
        leader_delay=0,
        front_delay=0,
        leader_tv=0,
    ):
        # 前馈控制
        Ui_sum = leader_a + front_a  # 前车和头车的加速度和

        # 有限时间控制
        ur = self.k1 * (front_dis - self.h * ego_v)  # 跟踪误差
        self.intOfUr = ur * self.Tstep  # 误差积分
        s = (front_dis - self.h * ego_v) + self.epsilon * self.intOfUr  # 滑模面
        S = self.q * s - front_delay  # 耦合滑模面

        # 误差观测器
        deltai1 = (
            -self.lanmda10
            * self.lip1 ** (1 / 3)
            * np.sign(self.dd1_hat - (front_dis - ego_v * self.Tstep))
            * np.abs(self.dd1_hat - (front_dis - ego_v * self.Tstep)) ** (2 / 3)
            + self.dd1_hat
        )
        self.dd1_hat += self.dd1_hat_dot * self.Tstep
        self.dd1_hat_dot = (
            -self.lanmda11
            * self.lip1 ** (1 / 2)
            * np.sign(self.dd1_hat - deltai1)
            * np.abs(self.dd1_hat - deltai1) ** (1 / 2)
        )

        deltai2 = (
            -self.lanmda20
            * self.lip2 ** (1 / 3)
            * np.sign(self.dd2_hat - ego_a)
            * np.abs(self.dd2_hat - ego_a) ** (2 / 3)
            + self.dd2_hat
        )
        self.dd2_hat += self.dd2_hat_dot * self.Tstep
        self.dd2_hat_dot = -self.lanmda21 * self.lip2 * np.sign(self.dd2_hat - deltai2)

        # 控制器
        Ui = (
            leader_v
            + front_v
            - 2 * (ego_v + self.dd1_hat)
            - self.epsilon * ur
            - self.delta / self.q * np.sign(S) * np.abs(S) ** self.ppp
        ) / (2 * self.h) - self.dd2_hat
        # Ui = np.clip(Ui, self.am, self.aM)  # 控制量约束
        print(Ui)
        return -Ui

# 初始化控制器
    

class CACCController:
    def __init__(self, tau_0, h_i, r_i, Q_i, k_i_init):
        self.tau_0 = tau_0  # estimate of uncertain dynamics parameter
        self.h_i = h_i      # headway time
        self.r_i = r_i      # standstill distance 
        self.Q_i = Q_i      # error cost matrix
        self.k_i = k_i_init # initial stabilizing error feedback gain
        self.P_i = None     # Riccati matrix
        self.data_x = []    # stored state data 
        self.data_w = []    # stored disturbance data
        
    def collect_data(self, x, w):
        self.data_x.append(x)
        self.data_w.append(w)
    
    def learn_optimal_gain(self):
        # Check rank condition 
        if len(self.data_x) > 9: # dimension of Ixx and Iwx
            Ixx = np.sum(np.array([np.outer(x,x) for x in self.data_x]), axis=0)  
            Iwx = np.sum(np.array([np.outer(w,x) for w,x in zip(self.data_w, self.data_x)]), axis=0)
            if np.linalg.matrix_rank(np.hstack((Ixx, Iwx))) == 9:
                # Solve for P_i and k_i 
                Theta = np.hstack((Ixx, 2*Iwx)) 
                Xi = np.sum(np.array([np.outer(x,x) for x in self.data_x]), axis=0) 
                vecQ = Xi @ np.reshape(self.Q_i + self.k_i.T @ self.k_i, (-1,1))
                sol = -np.linalg.pinv(Theta.T @ Theta) @ Theta.T @ vecQ
                self.P_i = np.reshape(sol[:6], (3,3)) 
                self.P_i = (self.P_i + self.P_i.T)/2  # ensure symmetry
                self.k_i = sol[-3:].flatten()
                # Clear data
                self.data_x = []  
                self.data_w = []
        
    def calc_speed(self, leader_v, leader_a, front_dis, front_v, front_a, ego_v, ego_a):
        # Calc error state x_i
        e_i = front_dis - self.r_i - self.h_i*ego_v
        de_i = front_v - ego_v - self.h_i*ego_a  
        dde_i = front_a - ego_a - self.h_i*(-ego_a/self.tau_0 + self.tau_0*leader_a/self.h_i 
                                             + 1/self.h_i*leader_a - self.k_i @ np.array([e_i,de_i,dde_i]))
        x_i = np.array([e_i, de_i, dde_i])
        
        # Collect data 
        self.collect_data(x_i, front_a)
        
        # Learn optimal gain
        self.learn_optimal_gain()
        
        # Calc desired accel 
        u_ai = -self.k_i @ x_i
        u_i = -1/self.h_i*ego_a + self.tau_0/self.h_i*front_a + 1/self.h_i*leader_a + self.tau_0/self.h_i*u_ai

        return u_i

class CACC_Sample_Model:
    def __init__(self, h=0.5, epsilon=1, delta=0.5, q=0.8, ppp=0.3, k1=0.15):
        self.h = h
        self.epsilon = epsilon
        self.delta = delta
        self.q = q
        self.ppp = ppp
        self.k1 = k1
        self.intOfUr = 0

    def calc_speed(self, leader_v, leader_a, front_dis, front_v, front_a, ego_v, ego_a, leader_delay=0, front_delay=0, leader_tv=0):
        # Feedforward control
        Vi_sum = leader_v + front_v
        DP = 2  # Number of preceding vehicles (leader and front vehicle)

        # Sliding mode control
        e1 = front_dis - self.h * (ego_v + 0)  # Assume dd1_hat is 0 for simplicity
        ur = self.k1 * e1
        self.intOfUr += ur * 0.01  # Assume a fixed time step of 0.01 seconds
        s = e1 + self.epsilon * self.intOfUr

        if front_delay == 0:
            S = self.q * s
        else:
            S = self.q * s - front_delay

        # Disturbance observer (simplified)
        dd1_hat = 0
        dd2_hat = 0
        dd11_hat = 0

        # Control law
        Ui = (Vi_sum - DP * (ego_v + dd1_hat) - self.epsilon * ur - self.delta / self.q * np.sign(S) * abs(S) ** self.ppp) / (DP * self.h) - dd2_hat - dd11_hat

        # Saturation limits (assuming a range of [-3, 3])
        Ui = max(min(Ui, 3), -3)

        return Ui+ ego_v



from collections import deque
class Adaptive_CACCController:
    def __init__(self, headway_time=0.5, speed_control_gain=-0.4, gap_control_gain_gap=-0.45,
                 gap_control_gain_gap_dot=0.125, collision_avoidance_gain_gap=0.45,
                 collision_avoidance_gain_gap_dot=0.05, emergency_threshold=2.0, headway_time_acc=1.0,
                 speed_control_min_gap=0.06, feedforward_gain=1.0, observer_gain=0.1, sliding_mode_gain=0.1,
                 spacing_err_queue_size=5, speed_err_queue_size=5):
        self.headway_time = headway_time
        self.speed_control_gain = speed_control_gain
        self.gap_control_gain_gap = gap_control_gain_gap
        self.gap_control_gain_gap_dot = gap_control_gain_gap_dot
        self.collision_avoidance_gain_gap = collision_avoidance_gain_gap
        self.collision_avoidance_gain_gap_dot = collision_avoidance_gain_gap_dot
        self.emergency_threshold = emergency_threshold
        self.headway_time_acc = headway_time_acc
        self.speed_control_min_gap = speed_control_min_gap
        self.feedforward_gain = feedforward_gain
        self.observer_gain = observer_gain
        self.sliding_mode_gain = sliding_mode_gain
        self.spacing_err_queue = deque(maxlen=spacing_err_queue_size)
        self.speed_err_queue = deque(maxlen=speed_err_queue_size)

    def calc_speed(self, leader_v, leader_a, front_dis, front_v, front_a, ego_v, ego_a,
                   leader_delay=0, front_delay=0, leader_tv=0, is_radar = False):
        # Feedforward control with nonlinear acceleration profile
        feedforward_acc = self.feedforward_gain * (leader_tv - leader_v)
        feedforward_acc *= math.tanh(leader_a * leader_delay) * (leader_tv - leader_v)
        feedforward_acc *= math.tanh(front_a * front_delay) * (front_v - leader_v)
        # print(feedforward_acc)
        # feedforward_acc = 0

        leading_v_err = ego_v - leader_v - leader_delay * leader_a
        spacing_err = front_dis - self.headway_time * ego_v - self.emergency_threshold
        speed_err = front_v - ego_v + self.headway_time * ego_a

        # Error observer with queue for spacing and speed errors
        self.spacing_err_queue.append(spacing_err)
        self.speed_err_queue.append(speed_err)
        observed_spacing_err = sum(self.spacing_err_queue) / len(self.spacing_err_queue)
        observed_speed_err = sum(self.speed_err_queue) / len(self.speed_err_queue)

        # Sliding mode control with adaptive gain adjustment
        sliding_mode_gain = math.tanh(abs(observed_spacing_err) / self.speed_control_min_gap) * self.sliding_mode_gain
        # sliding_mode_gain += math.tanh(abs(observed_speed_err) / self.speed_control_min_gap) * self.sliding_mode_gain

        # Dynamic gain adjustment
        gap_control_gain_gap = self.gap_control_gain_gap * (1 + sliding_mode_gain)
        gap_control_gain_gap_dot = self.gap_control_gain_gap_dot * (1 + sliding_mode_gain)
        speed_control_gain = self.speed_control_gain

        if abs(spacing_err) > self.speed_control_min_gap:
            gap_control_gain_gap *= (1 + sliding_mode_gain)
            gap_control_gain_gap_dot *= (1 + sliding_mode_gain)
        else:
            speed_control_gain *= (1 + sliding_mode_gain)

        # Safety constraints (remains the same as before)
        if front_dis < self.emergency_threshold:
            feedforward_acc = -5.0
            speed_control_gain = 0.0
            gap_control_gain_gap_dot = 0.0

        return (ego_v + feedforward_acc +
                speed_control_gain * leading_v_err -
                gap_control_gain_gap * observed_spacing_err +
                gap_control_gain_gap_dot * observed_speed_err)