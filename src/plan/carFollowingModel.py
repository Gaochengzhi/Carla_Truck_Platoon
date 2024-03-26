import numpy as np
import math

class CACC_Model:
    def __init__(self, L=19.56, t_0=0.5,s_0 = 5):
        pass
    def calc_speed(self, 
                leader_v,
                leader_a,
                front_dis,
                front_v,
                front_a,
                ego_v,
                ego_a):
        pass




class IDM():
    def __init__(self, speed_limit=30, max_acc=5, normal_decel=5, delta=4, safe_distance=5.5, human_reaction_time=0.1, max_brake_decel=6.0):
        self.v0 = speed_limit
        self.delta = delta
        self.a = max_acc
        self.b = normal_decel
        self.max_brake_decel = max_brake_decel
        self.s0 = safe_distance
        self.T = human_reaction_time
        self.sqrtab = np.sqrt(max_acc*normal_decel)

    def calc_acc(self, front_v, front_dis, ego_v):
        self.s0 = max(ego_v*1.5, 3)
        if front_dis < 4:
            return -self.max_brake_decel
        delta_v = ego_v-front_v
        s_star_raw = self.s0 + ego_v * self.T\
            + (ego_v * delta_v) / (2 * self.sqrtab)
        s_star = max(s_star_raw, self.s0)
        acc = self.a * (1 - np.power(ego_v / self.v0,
                        self.delta) - (s_star ** 2) / (front_dis**2))
        acc = max(acc, -self.max_brake_decel)
        return acc


class PFL_CACC:
    def __init__(self, c_p=4.1, c_v=0.23, c_a=0.1, k_v=0.5, k_a=0.2, s_0=0.5):
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
    def __init__(self, c_p=1.1, c_v=0.23, c_a=0.07, k_v=0.5, k_a=0.2, s_0=0.5):
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

        u = self.c_p * spacing_error + self.c_v * velocity_error_front + self.c_a * acceleration_error_front
           
        return u+ego_v


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
        e = back_dis - self.s_0 * ego_v-2
        # e = back_dis - 5
        speed = ego_v+ self.k1 * e + self.k2 * (target_v - ego_v)
        return target_v
        # return 20


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
        if time_gap > self.headway_time+0.2 or abs(spacing_err) > self.speed_control_min_gap or time_gap < self.headway_time - 0.2:
            return ego_v + self.gap_control_gain_gap * spacing_err + self.gap_control_gain_gap_dot * speed_err
        elif spacing_err < -self.speed_control_min_gap:
            return ego_v + self.collision_avoidance_gain_gap * spacing_err + self.collision_avoidance_gain_gap_dot * speed_err
        else:
            return ego_v + self.speed_control_gain * leading_v_err 

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
from scipy.optimize import minimize
import math

class Adaptive_CACCController:
    def __init__(self, headway_time=0.5, speed_control_gain=-0.4, gap_control_gain_gap=0.45,
                 gap_control_gain_gap_dot=0.0125, collision_avoidance_gain_gap=0.45,
                 collision_avoidance_gain_gap_dot=0.05, emergency_threshold=2.0, headway_time_acc=1.0,
                 speed_control_min_gap=3.66):
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
        leading_v_err = ego_v - leader_v - leader_delay*leader_a
        time_gap = front_dis / max(ego_v, 0.1)
        spacing_err = front_dis - self.headway_time * ego_v - self.emergency_threshold
        speed_err = front_v - ego_v + self.headway_time * ego_a
        if abs(spacing_err) > self.speed_control_min_gap:
            self.gap_control_gain_gap = math.tanh(abs(spacing_err)/10)
            return ego_v + self.gap_control_gain_gap * spacing_err + self.gap_control_gain_gap_dot * speed_err
        else:
            return ego_v + self.speed_control_gain * leading_v_err + self.gap_control_gain_gap_dot *spacing_err
    

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