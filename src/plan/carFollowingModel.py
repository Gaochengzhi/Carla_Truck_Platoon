import numpy as np


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
    def __init__(self, c_p, c_v, c_a, k_v, k_a, s_0=0.5):
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

    def calc_speed(self, front_x, front_v, front_a, leader_v, leader_a, ego_x, ego_v, ego_a):
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
        spacing_error = front_x - ego_x - self.L
        velocity_error_front = front_v - ego_v
        acceleration_error_front = front_a - ego_a
        velocity_error_leader = leader_v - ego_v
        acceleration_error_leader = leader_a - ego_a

        u = self.c_p * spacing_error + self.c_v * velocity_error_front + self.c_a * acceleration_error_front \
            + self.k_v * velocity_error_leader + self.k_a * acceleration_error_leader
        return u


class BDL_Controller:
    def __init__(self, k_s=4.5, k_v=6, L=10.54, h=0.5, d_0=1):
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
        self.d_0 = d_0

    def calc_speed(self, front_dis, back_dis, ego_v, leader_v):
        """
        Calculate the control input for the BDL controller.
        :param front_x: position of the preceding vehicle
        :param back_x: position of the following vehicle
        :param ego_x: position of the ego vehicle
        :param ego_v: velocity of the ego vehicle
        :return: control input u
        """
        spacing_error_front = front_dis - self.L - self.h * leader_v - self.d_0
        spacing_error_back = back_dis - self.L - self.h * leader_v - self.d_0
        velocity_error_leader = leader_v - ego_v
        u = self.k_s * spacing_error_front - self.k_s * \
            spacing_error_back + self.k_v * velocity_error_leader
        return u
