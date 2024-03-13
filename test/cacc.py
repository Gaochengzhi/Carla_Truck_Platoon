import pandas as pd
import numpy as np
import math

'''==Parameters for CACC model=='''
thw_cacc = 0.60                          # The desired time gap for CACC vehicles
len_veh = 5
dm = 3.4
kp = 0.45
kd = 0.0125
kp_c = 0.005
kd_c = 0.05

'''==Parameters for ACC model=='''
thw_acc = 1.10                           # The desired time gap for ACC vehicles
k1 = 0.23
k2 = 0.07
k1_c = 0.04
k2_c = 0.8

'''==Parameters for IDM=='''
thw_mdv = 1.50                           # The desired time gap for manually driven vehicles
b = 1.5
s0 = 2.0
v0 = 33.3

'''==Parameters for Cruise mode=='''
k0 = 0.40

'''==Parameters for simulation setting=='''
len_link = 5000
set_area = [1000, 3000]
initial_speed = 20                       # The initial speed
simu_step = 0.1                          # The simulation step
veh_gap = 30                             # The initial headway
max_a = 2                                # The maximum acceleration
min_a = -4                               # The maximum deceleration
max_speed = 35                           # The maximum speed
min_speed = 0                            # The minimum speed
Threhold = [thw_mdv, thw_cacc, thw_acc]  # The TTC threshold
# The perception-reaction time of manually driven vehicle, CACC vehicle, ACC vehicle
td = [1.20, 0.10, 0.60]

'''==Parameters for transition mode=='''
KT_CA = 15                               # The duration of transition from CACC mode to ACC mode
# The duration of transition from ACC mode to CACC mode
KT_AC = 15

'''==Gaperror for controller switch=='''
errorthre = 0.01

'''==Initialize platoon=='''


def Initial():
    line = []                            # The platoon
    location = 0
    i = 0
    while (i < 10):
        location += veh_gap
        i += 1
        line.append([location, initial_speed, 0, 1, 1, 0, 0, 1])
        '''
        each element represent one vehicle, the definitions are defined as follows:
        # veh[0]: Location, veh[1]: Speed, veh[2]: Acceleration
        # veh[3]: The initial vehicle type, veh[4]: Updated vehicle type in the simulation (0-MDV, 1-CACC vehilce, 2- ACC vehicle)
        # veh[5]: Operating time for transition mode from CACC to ACC, veh[6]: Operating time for transition mode from ACC to CACC
        # veh[7]: Control mode (0-MDV, 1-gap regulating mode, 2-gap closing mode
        '''
    for id in mdv:                       # Provides attributes for MDV
        line[id][3] = 0
        line[id][4] = 0
        line[id][7] = 0
    if improve == 0:                     # Vehicle following the MDV degrades to ACC vehicles
        for i in range(0, len(line) - 1):
            if line[i][3] == 1 and line[i+1][3] == 0:
                line[i][3] = 2
                line[i][4] = 2
    return line


'''==Realistic ACC mode=='''


def ACC(back_veh, fore_veh, t_acc):
    s = fore_veh[0] - back_veh[0] - len_veh
    ind = back_veh[7]
    nextind = 0
    if ind == 1:
        if s <= thw_acc * initial_speed * 1.20:               # gap-regulating mode
            a_ACC = k1 * (fore_veh[0] - back_veh[0] - t_acc *
                          back_veh[1] - len_veh) + k2 * (fore_veh[1] - back_veh[1])
            nextind = 1
        else:                                                 # controller switch from gap-regulating mode to gap-closing mode
            a_ACC = k1_c * (fore_veh[0] - back_veh[0] - t_acc *
                            back_veh[1] - len_veh) + k2_c * (fore_veh[1] - back_veh[1])
            nextind = 2
    else:
        gaperror = fore_veh[0] - back_veh[0] - t_acc * back_veh[1] - len_veh
        if gaperror < errorthre:                              # gap closing mode
            a_ACC = k1 * (fore_veh[0] - back_veh[0] - t_acc *
                          back_veh[1] - len_veh) + k2 * (fore_veh[1] - back_veh[1])
            nextind = 1
        else:                                                 # controller swithch from gap-closing mode to gap-regulating mode
            a_ACC = k1_c * (fore_veh[0] - back_veh[0] - t_acc *
                            back_veh[1] - len_veh) + k2_c * (fore_veh[1] - back_veh[1])
            nextind = 2
    dv_ACC = a_ACC * simu_step
    return dv_ACC, nextind


'''==Realistic CACC mode=='''


def CACC(back_veh, fore_veh, t_cacc):
    s = fore_veh[0] - back_veh[0] - len_veh
    ind = back_veh[7]
    nextind = 0
    gaperror = fore_veh[0] - back_veh[0] - len_veh - t_cacc * back_veh[1]
    if ind == 1:
        if s <= thw_cacc * initial_speed * 1.20:             # gap-regulating mode
            dv_CACC = kp * gaperror + kd * \
                (fore_veh[1] - back_veh[1] - t_cacc * back_veh[2])
            nextind = 1
        else:                                                # controller switch from gap-regulating mode to gap-closing mode
            dv_CACC = kp_c * gaperror + kd_c * \
                (fore_veh[1] - back_veh[1] - t_cacc * back_veh[2])
            nextind = 2
    else:
        if gaperror < errorthre:                             # gap closing mode
            dv_CACC = kp * gaperror + kd * \
                (fore_veh[1] - back_veh[1] - t_cacc * back_veh[2])
            nextind = 1
        else:                                                # controller switch from gap-regulating mode to gap-closing mode
            dv_CACC = kp_c * gaperror + kd_c * \
                (fore_veh[1] - back_veh[1] - t_cacc * back_veh[2])
            nextind = 2
    return dv_CACC, nextind


'''==Intelligent driver model=='''


def MDV(back_veh, fore_veh):
    temp1 = back_veh[1] * thw_mdv + \
        (back_veh[1] * (back_veh[1] - fore_veh[1]) / 2 / math.sqrt(max_a * b))
    s1 = s0 + max([0, temp1])
    temp2 = max_a * (1 - (back_veh[1] / v0) ** 4 -
                     (s1 / (fore_veh[0] - back_veh[0] - len_veh)) ** 2)
    dv_MDV = max([min_a, temp2]) * simu_step
    return dv_MDV


'''==Cruise mode=='''


def Cruise(sub_vehicle):
    dv_cruise = k0 * (initial_speed - sub_vehicle[1]) * simu_step
    return dv_cruise


'''==Calculate the time gap for the transition mode=='''


# Transition from mode1 to mode2
def TransitionGap(mode1, mode2, t):
    if mode1 == 'CACC' and mode2 == 'ACC':                  # Transition from CACC mode to ACC mode
        if t > KT_CA:
            thw_T = thw_acc
        else:
            thw_T = thw_cacc + (1 / KT_CA) * (thw_acc - thw_cacc) * t
    if mode1 == 'ACC' and mode2 == 'CACC':                  # Transition from ACC mode to CACC mode
        if t > KT_AC:
            thw_T = thw_cacc
        else:
            thw_T = thw_acc + (1 / KT_AC) * (thw_cacc - thw_acc) * t
    return thw_T


'''==Limitations on acceleration=='''


def AcceleCorrect(value):
    if value > max_a * simu_step:
        value = max_a * simu_step
    if value < min_a * simu_step:
        value = min_a * simu_step
    return value


'''==Limitations on speed=='''


def SpeedCorrect(speed):
    if speed > max_speed:
        speed = max_speed
    if speed < min_speed:
        speed = min_speed
    return speed


'''==Update the vehicle dynamic in one simulation step=='''


def LaneUpdation(lane, ind):
    # Store the updated speed change (acceleration * simulation step)
    update_dv = [0 for i in range(len(lane))]
    if ind != 0:
        for i in range(len(lane) - 1):
            # The degradation from CACC mode to ACC mode
            if lane[i][0] >= set_area[0] and lane[i][0] <= set_area[1] and lane[i][3] == 1:
                lane[i][4] = 2
                lane[i][5] += simu_step
            # The upgradation from ACC mode to CACC mode
            if lane[i][0] > set_area[1] and lane[i][3] == 1:
                lane[i][4] = 1
                lane[i][6] += simu_step

    for i in range(len(lane) - 1):
        back_veh = lane[i]
        fore_veh = lane[i + 1]
        if lane[i][4] == 0:                                  # Update the dynamics of MDV
            cruise_dv = MDV(back_veh, fore_veh)
            cruise_ind = 0

        if lane[i][3] == 1 and lane[i][4] == 2:              # Update the dynamics of CACC
            Tacc = TransitionGap("CACC", "ACC", lane[i][5])
            cruise_dv, cruise_ind = ACC(back_veh, fore_veh, Tacc)

        if lane[i][3] == 1 and lane[i][4] == 1 and lane[i][6] == 0:
            cruise_dv, cruise_ind = CACC(back_veh, fore_veh, thw_cacc)

        if lane[i][3] == 1 and lane[i][4] == 1 and lane[i][6] > 0:
            Tcacc = TransitionGap("ACC", "CACC", lane[i][6])
            cruise_dv, cruise_ind = CACC(back_veh, fore_veh, Tcacc)

        # Update the dynamics of initial ACC vehicle
        if lane[i][3] == 2:
            cruise_dv, cruise_ind = ACC(back_veh, fore_veh, thw_acc)

        update_dv[i] = AcceleCorrect(cruise_dv)
        lane[i][7] = cruise_ind

    update_dv[-1] = AcceleCorrect(Cruise(lane[-1]))

    for i in range(len(update_dv)):                         # Update the platoon
        lane[i][2] = update_dv[i] / simu_step
        lane[i][1] = SpeedCorrect(lane[i][1] + update_dv[i])
        lane[i][0] = lane[i][0] + lane[i][1] * simu_step
    return lane


'''==Preheating simulation=='''


def Fleet(lane):
    # The preheating time is 600s
    for runtime in range(6000):
        LaneUpdation(lane, 0)
    start_x = lane[-1][0]
    # Initialize vehicle position, make the first vehicle located at the start point of simulation segment
    for veh in lane:
        veh[0] = veh[0] - start_x
    return lane


'''==Calculate surrogate safety measures=='''


def CalIndex(line):
    TERC_F = [0 for i in range(10)]
    TIT_F = [0 for i in range(10)]
    for i in range(len(line)-1):
        ind = line[i][4]
        SSDL = line[i + 1][0] - line[i][0] - \
            len_veh + (line[i + 1][1]) ** 2 / 2 / dm
        SSDF = line[i][1] * td[ind] + line[i][1] ** 2 / 2 / dm
        if line[i][1] > 0:
            TTC_brake = (line[i + 1][0] - line[i][0] - len_veh) / line[i][1]
        else:
            TTC_brake = float('inf')
        TTC_thre = Threhold[ind]
        if (SSDL - SSDF) <= 0:
            TERC_F[i] += simu_step
        if (TTC_brake <= TTC_thre and TTC_brake >= 0):
            TIT_F[i] += simu_step * (1 / TTC_brake - 1 / TTC_thre)
    return TERC_F, TIT_F


'''==Calculate the SD of speed=='''


def VehSD(Speed_list):
    SD_F = []
    with open("SD_ALL.txt", "w") as SDtoTXT:
        for j in range(10):
            # Extract the speed of each vehicle
            unit_speed = [Speed_list[j][i] for i in range(len(Speed_list[1]))]
            SDtoTXT.write(str(format(np.std(unit_speed), ".4f")) + "\n")
            SD_F.append(np.std(unit_speed))
        print("Average SD: %f" % np.average(SD_F))


'''==Calculate the absolute value of acceleration=='''


def VehAccele(Accele_list):
    Accele_F = []
    with open("AC_ALL.txt", "w") as SDtoTXT:
        for j in range(10):
            # Extract the absolute acceleration of each vehicle
            unit_accele = [abs(Accele_list[j][i])
                           for i in range(len(Accele_list[1]))]
            SDtoTXT.write(str(format(np.average(unit_accele), ".4f")) + "\n")
            Accele_F.append(np.average(unit_accele))
        print("Average AC: %f" % np.average(Accele_F))


'''==Write to txt=='''


def Write_txt(name, list1):
    with open(name, "w") as f_txt:
        for item in list1:
            f_txt.write(str(item) + "\n")


'''==Write to excel=='''


def Toexcel(name, data):
    matrix = pd.DataFrame(data, index=range(1, 11)).stack().unstack(0)
    matrix.columns = ["1st", "2nd", "3rd", "4th",
                      "5th", "6th", "7th", "8th", "9th", "10th"]
    matrix.to_excel(name)


'''==Total simulation process=='''


def SingleProcess():
    # Initialize lists
    TERC_all = [0 for i in range(10)]
    TIT_all = [0 for i in range(10)]
    Location = [[] for i in range(10)]
    Speed = [[] for i in range(10)]
    Acceleration = [[] for i in range(10)]
    States = [[] for i in range(10)]

    # Generate a platoon
    lane = Initial()
    # Preheating simulation
    fleet = Fleet(lane)
    # Output the initial platoon
    Write_txt("0_lane.txt", fleet)

    # Extract the basic variables
    while (fleet[-1][0] < len_link):
        fleet = LaneUpdation(fleet, 1)
        terc_f, tit_f = CalIndex(fleet)

        for index in range(10):
            Location[index].append(fleet[9 - index][0])
            Speed[index].append(fleet[9 - index][1])
            Acceleration[index].append(fleet[9 - index][2])
            States[index].append(fleet[9 - index][7])
            TERC_all[index] += terc_f[9 - index]
            TIT_all[index] += tit_f[9 - index]

    # Output the platoon after simulation
    Write_txt("1_lane.txt", fleet)

    # Calculate and print the surrogate safety measures
    print("Average TERCRI: %f" % float(np.average(TERC_all)))
    print("Average TIT: %f" % float(np.average(TIT_all)))
    VehSD(Speed)
    VehAccele(Acceleration)

    # Write the results to files
    Write_txt("TIT_ALL.txt", TIT_all)
    Write_txt("TERC_ALL.txt", TERC_all)
    Toexcel("Speed.xlsx", Speed)
    Toexcel("Location.xlsx", Location)
    Toexcel("Acceleration.xlsx", Acceleration)
    Toexcel("States.xlsx", States)
    print("#########Success！##########")


'''==Main function=='''
if __name__ == "__main__":
    # input “-n”, represting the nth vehicle is MDV; Empty list represents the full CACC platoon
    mdv = []
    # The index respresent whether the MDV equips information-sending devices, 0-no, 1-yes
    improve = 0
    SingleProcess()                                             # Simulation runs
