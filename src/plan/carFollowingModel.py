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
        self.s0 = max(ego_v*0.5, 3)
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

class ACC:
    def __init__(self, s_0=2, t_gap=1.5, k_1=0.4, k_2=0.1):
        self.s_0 = s_0
        self.t_gap = t_gap
        self.k_1 = k_1
        self.k_2 = k_2

    def calc_spacing(self, v):
        return self.s_0 + v * self.t_gap

    def calc_acc(self, v_leader, v_follower, s):
        s_des = self.calc_spacing(v_follower)
        acc = self.k_1 * (v_leader - v_follower) + self.k_2 * (s - s_des)
        return acc

class CACC(ACC):
    def __init__(self, s_0=2, t_gap=0.5, k_1=0.4, k_2=0.1, k_3=0.2):
        super().__init__(s_0, t_gap, k_1, k_2)
        self.k_3 = k_3

    def calc_acc(self, v_leader, v_follower, s, a_leader):
        s_des = self.calc_spacing(v_follower)
        acc = self.k_1 * (v_leader - v_follower) + self.k_2 * (s - s_des) + self.k_3 * a_leader
        return acc