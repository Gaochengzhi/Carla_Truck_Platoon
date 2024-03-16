import numpy as np

class CACCModel:
    def __init__(self, L=19.56, t_0=0.5,s_0 = 5):
        pass
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

    def calc_acc(self, front_v, distance_s, ego_v):
        self.s0 = max(ego_v*1.5, 3)
        if distance_s < 4:
            return -self.max_brake_decel
        delta_v = ego_v-front_v
        s_star_raw = self.s0 + ego_v * self.T\
            + (ego_v * delta_v) / (2 * self.sqrtab)
        s_star = max(s_star_raw, self.s0)
        acc = self.a * (1 - np.power(ego_v / self.v0,
                        self.delta) - (s_star ** 2) / (distance_s**2))
        acc = max(acc, -self.max_brake_decel)
        return acc


class PLF_Controller:
    def __init__(self, c_p=2.5, c_v=0.2, c_a=0.1, k_v=0.1, k_a=0.1, s_0=0.5):
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

    def calc_speed(self, front_dis, front_v, front_a, leader_v, leader_a,  ego_v, ego_a):
        """
        Calculate the control input for the PLF controller.

        :param front_x: position of the preceding vehicle
        :param front_v: velocity of the preceding vehicle
        :param front_a: acceleration of the preceding vehicle
        :param leader_v: velocity of the leading vehicle
        :param leader_a: acceleration of the leading vehicle
        :param ego_x: position of the ego vehicle
        :param ego_v: velocity of the ego vehicle
        :param ego_a: acceleration of the ego vehicle
        :return: control input u
        """
        spacing_error = front_dis - self.s_0 * ego_v 
        velocity_error_front = front_v - ego_v
        acceleration_error_front = front_a - ego_a
        velocity_error_leader = leader_v - ego_v
        acceleration_error_leader = leader_a - ego_a

        u = self.c_p * spacing_error + self.c_v * velocity_error_front + self.c_a * acceleration_error_front \
            + self.k_v * velocity_error_leader + self.k_a * acceleration_error_leader
        return u


class BDL_Controller:
    def __init__(self, k_s=0.45, k_v=0.2, L=20.54, h=0.5, s_0=5):
        """
        Initialize the BDL controller.
        :param k_s: spacing error gain
        :param k_v: velocity error gain between the ego vehicle and the leading vehicle
        :param L: length of the ego vehicle
        :param h: time headway
        :param v_0: velocity of the leading vehicle
        :param D: desired spacing at standstill
        """
        self.k_s = k_s
        self.k_v = k_v
        self.L = L
        self.h = h
        self.d_0 = s_0

    def calc_speed(self, front_dis, back_dis, ego_v, leader_v):
        """
        Calculate the control input for the BDL controller.
        :param front_x: position of the preceding vehicle
        :param back_x: position of the following vehicle
        :param ego_x: position of the ego vehicle
        :param ego_v: velocity of the ego vehicle
        :return: control input u
        """
        spacing_error_front = front_dis  - self.h * leader_v - self.d_0
        spacing_error_back = back_dis  - self.h * leader_v - self.d_0
        velocity_error_leader = leader_v - ego_v
        u = self.k_s * spacing_error_front - self.k_s * \
            spacing_error_back + self.k_v * velocity_error_leader
        return u

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

    def calc_acc(self, target_v, ego_v, back_dis):
        """
        Calculate the control input for the ACC controller.
        :param vi_1: velocity of the preceding vehicle
        :param vi: velocity of the ego vehicle
        :param xi_1: position of the preceding vehicle
        :param xi: position of the ego vehicle
        :return: control input ai
        """
        e = back_dis - self.s_0 * ego_v 
        ai = self.k1 * e + self.k2 * (target_v - ego_v)
        return min(ai, 33)
class Path_CACC:
    def __init__(self, k0=1.1, k1=0.23, k2=0.07, k3=0.45, T=0.5):
        """
        Initialize the CACC controller.

        :param k0: control input gain of the preceding vehicle
        :param k1: spacing error gain
        :param k2: relative velocity gain
        :param k3: velocity error gain
        :param T: desired time gap
        """
        self.k0 = k0
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3
        self.T = T

    def calc_speed(self, front_dis,front_v, ego_speed, front_a):
        """
        Calculate the control input for the CACC controller.

        :param vi_1: velocity of the preceding vehicle
        :param vi: velocity of the ego vehicle
        :param xi_1: position of the preceding vehicle
        :param xi: position of the ego vehicle
        :param ai_1: control input of the preceding vehicle
        :return: control input ai
        """
        e = front_dis - self.T * ego_speed
        ai = self.k0 * front_a + self.k1 * e + self.k2 * (front_v - ego_speed) + self.k3 * e
        return ai