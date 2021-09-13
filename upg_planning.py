import numpy as np

from upg_kinetic import *
from upg_mass_center import *
from upg_tools import *

MIN_RADIUS = 2500
MAX_RADIUS = 15000

TRAJ_ACCUR = 50  # useless now
DSTEP = 50
STEP_HEIGHT = 0
EPSILON = 20  # accuracy asked in movement


def cmd_joystick(d, r):
    """
    Traite les données reçues des commandes joystick pour paramétrer les trajectoires des 4 jambes.
    On suppose un input non nul : (d, r) != ((0, 0), 0)

    :param d: vecteur unitaire de direction
    :param r: facteur de rotation (-1 < r < 1)
    :return: centre de rotation, direction de déplacement.
    >>> cmd_joystick((1, 0), 0)
    (None, (1, 0))
    >>> cmd_joystick((0, 1), 0)
    (None, (0, 1))
    >>> cmd_joystick((1, 0), 1)
    ((0, 2500), (1, 0))
    >>> cmd_joystick((0, 1), -1)
    ((2500, 0), (0, 1))
    >>> cmd_joystick((np.sqrt(3)/2, 1/2), -0.5)
    ((4375.0, -7577.722283113838), (0.8660254037844386, 0.5))
    """
    if d[0] == 0 and d[1] == 0:
        # rotation sur lui-même
        c = (0, 0)
    elif r == 0:
        # marche tout droit
        c = None
    else:
        # marche courbe standard
        n = normal_vector(d)
        signe_r = (r > 0) - (r < 0)
        c = signe_r * (n[0] * MAX_RADIUS - signe_r * r * n[0] * (MAX_RADIUS - MIN_RADIUS)), \
            signe_r * (n[1] * MAX_RADIUS - signe_r * r * n[1] * (MAX_RADIUS - MIN_RADIUS))
    return c, d


############################## RELATIVE WAY ##################################


def compute_traj_form_joystick_rel(joystick):
    """
    A partir des valeurs retournées par cmd_joystick, calcul les trajectoires circulaires de chacunes des pattes

    :param joystick: position du centre de rotation, direction
    :return: les quatres trajectoires
    """
    centre, direction = joystick[0], joystick[1]
    direction = unitary_vec(direction)
    print(joystick)
    traj = []
    r = []
    V = get_verins_12()
    pos = direct_rel_12(V)
    # Computing radius
    for leg in range(4):
        radius = np.sqrt((pos[leg * 3 + 0] - centre[0]) ** 2 + (pos[leg * 3 + 1] - centre[1]) ** 2)
        r.append(radius)
    r_max = max(r)
    n = int(2 * np.pi * r_max / TRAJ_ACCUR)
    for i in range(n - 10):
        L = []
        for leg in range(4):
            alpha = np.arccos(abs((centre[1] - pos[3 * leg + 1])) / r[leg])
            signe_cx = (centre[0] - pos[leg * 3 + 0]) / abs(centre[0] - pos[leg * 3 + 0])
            signe_cy = (centre[1] - pos[leg * 3 + 1]) / abs(centre[1] - pos[leg * 3 + 1])
            if signe_cx < 0 and signe_cy < 0:
                alpha = + np.pi / 2 - alpha
            if signe_cx > 0 and signe_cy < 0:
                alpha = + np.pi / 2 + alpha
            if signe_cx < 0 and signe_cy > 0:
                alpha = - np.pi / 2 + alpha
            if signe_cx > 0 and signe_cy > 0:
                alpha = - np.pi / 2 - alpha
            L = np.append(L, (r[leg] * np.cos((2 * i * np.pi) / n + alpha) + centre[0],
                              r[leg] * np.sin((2 * i * np.pi) / n + alpha) + centre[1],
                              pos[3 * leg + 2]))
        traj.append(L)
    return traj


def is_accessible_rel(leg_id, point):
    """
    Calcule l'accessibilité d'un point dans le repère 3D de la jambe.

    :param leg_id: ID de la jambe
    :param point: point cible
    :return: True ou False + valeur des verins pour atteindre ce point
    """
    lpl = ROBOT['legs'][leg_id]['lengths']
    v1, v2, v3 = 535, 615, 520
    x0, y0, z0 = direct_rel_3(v1, v2, v3, leg_id)
    xt, yt, zt = point
    dx, dy, dz = xt - x0, yt - y0, zt - z0
    traj = []
    n = 40
    for i in range(n):
        traj.append(np.array([x0 + i * dx / n,
                              y0 + i * dy / n,
                              z0 + i * dz / n]))
    Verins = move_leg(traj, v1, v2, v3, leg_id, display=False, upgrade=False, solved=True)
    res = [Verins[n - 1][0], Verins[n - 1][1], Verins[n - 1][2]]
    acces = True
    xf, yf, zf = direct_rel_3(res[0], res[1], res[2], leg_id)
    # print("x : ", x0, xt, traj[n-1][0], xf)
    # print("y : ", y0, yt, traj[n-1][1], yf)
    # print("z : ", z0, zt, traj[n-1][2], zf)
    # print("V = ", res)
    if distance([xt, yt, zt], [xf, yf, zf]) > 50:
        acces = False
    return acces, res


def furthest_accessible_rel(traj, leg_id):
    """
    Compute the maximum step size following the trajectory, depending on the accessible zone for the leg.
    traj should be the trajectory of all elgs (basically returned by compute_traj_from_joystick)

    :param traj: trajectory to follow
    :param leg_id: ID of the leg
    :return: the furthest point accessible from the traj
    """
    v1, v2, v3 = get_verins_3(leg_id)
    xt, yt, zt = traj[0][leg_id * 3 + 0:leg_id * 3 + 3]
    lpl = ROBOT['legs'][leg_id]['lengths']
    for i in range(1, len(traj)):
        # get data
        x0, y0, z0 = direct_rel_3(v1, v2, v3, leg_id)
        if distance((x0, y0, z0), (xt, yt, zt)) > 20:  # exit condition
            return i
        xt, yt, zt = traj[i][leg_id * 3 + 0], traj[i][leg_id * 3 + 1], traj[i][leg_id * 3 + 2]
        dX = np.array([xt - x0, yt - y0, zt - z0])
        # solve
        J = ROBOT['legs'][leg_id]['matrix'].T @ gen_jacob_3(v1 / 1000, v2 / 1000, v3 / 1000,
                                                            np.arccos(v3_to_cos_angle(v3, lpl)), lpl)
        P = J.T @ J
        q = - J.T @ dX
        lb = np.array([450.0 - v1, 450.0 - v2, 450.0 - v3])
        ub = np.array([650.0 - v1, 650.0 - v2, 650.0 - v3])
        dV = solve_qp(P, q, lb=lb, ub=ub)
        v1, v2, v3 = v1 + dV[0], v2 + dV[1], v3 + dV[2]
    return len(traj)


def furthest_accessible_step_all_legs_rel(traj, step_height):
    """
    Search for the furthest points of trajectories accessible for each legs, taking into account the height of the step

    :param traj: trajectory of the 4 legs
    :param step_height: height of one step
    :return: array of the 4 index of traj
    """
    V = get_verins_12()
    init = traj[0].copy()
    for j in range(step_height):
        traj.insert(0, init.copy())
        for leg in range(4):
            traj[0][leg * 3 + 2] += step_height + 1 - j
    for k in range(step_height, len(traj)):
        for leg in range(4):
            traj[k][leg * 3 + 2] += step_height

    max_step = []
    for leg in range(4):
        step = furthest_accessible_rel(traj, leg) - (step_height + 1)
        max_step.append(step)
    return max_step


############################### ABSOLUTE WAY ##################################


def compute_traj_from_joystick_leg_equals_distance(joystick, leg_id):
    """
    Compute the trajectory for one leg with a constant distance between each points of the discretization

    :param joystick: ((rota_center_x, rota_center_y), (dir_x, dir_y))
    :param leg_id: ID of the leg
    :return:
    """
    centre, direction = joystick[0], joystick[1]
    # direction = unitary_vec(direction)
    pos = direct_abs(get_verins_12(), get_O(), get_omega())
    radius = distance(pos[leg_id * 3 + 0:leg_id * 3 + 3], (centre[0], centre[1], 0))
    n = int((2 * np.pi * radius) / DSTEP)
    traj_leg = []
    for i in range(n):
        alpha = np.arccos(abs((centre[1] - pos[3 * leg_id + 1])) / radius)
        signe_cx = (centre[0] - pos[leg_id * 3 + 0]) / abs(centre[0] - pos[leg_id * 3 + 0])
        signe_cy = (centre[1] - pos[leg_id * 3 + 1]) / abs(centre[1] - pos[leg_id * 3 + 1])
        if signe_cx < 0 and signe_cy < 0:
            alpha = + np.pi / 2 - alpha
        if signe_cx > 0 > signe_cy:
            alpha = + np.pi / 2 + alpha
        if signe_cx < 0 < signe_cy:
            alpha = - np.pi / 2 + alpha
        if signe_cx > 0 and signe_cy > 0:
            alpha = - np.pi / 2 - alpha
        traj_leg.append([radius * np.cos((2 * i * np.pi) / n + alpha) + centre[0],
                         radius * np.sin((2 * i * np.pi) / n + alpha) + centre[1],
                         pos[3 * leg_id + 2]])
        # if len(traj_leg) == 0:
        #     traj_leg = [radius * np.cos((2 * i * np.pi) / n + alpha) + centre[0],
        #                 radius * np.sin((2 * i * np.pi) / n + alpha) + centre[1],
        #                 pos[3 * leg_id + 2]]
        # else:
        #     traj_leg = np.vstack((traj_leg, (radius * np.cos((2 * i * np.pi) / n + alpha) + centre[0],
        #                                      radius * np.sin((2 * i * np.pi) / n + alpha) + centre[1],
        #                                      pos[3 * leg_id + 2])))
    return traj_leg


def compute_traj_form_joystick_abs_equal_dist(joystick):
    """
    Compute all 4 trajectories for each leg with a constant distance between each points of the discretization

    :param joystick: ((rota_center_x, rota_center_y), (dir_x, dir_y))
    :return:
    """
    traj = []
    r = []
    pos = direct_abs(get_verins_12(), get_O(), get_omega())
    for leg in range(4):
        L = compute_traj_from_joystick_leg_equals_distance(joystick, leg)
        traj.append(L)
    return traj


def compute_step(traj, leg_id, max_omega=10, const_omega=True, reg_val=0.01):
    """
    Assuming that traj[0] is the actual position of the leg, compute the maximum step possible following its trajectory
    !!! DON'T WORK !!!

    :param traj: trajectory of the leg
    :param leg_id: ID of the leg
    :return: list of cylinder's elongations and O, omega displacement, and modified trajectory raising the leg
    """
    # n_step = len(traj[leg_id])
    # traj_leg = [np.zeros(12)] * n_step
    # for i in range(n_step):
    #     for leg in range(4):
    #         if leg != leg_id:
    #             traj_leg[leg*3:leg*3+3] = [0, 0, 0]
    #         else:
    #             traj_leg[leg*3:leg*3+3] = traj[leg_id][i]
    #             traj_leg[leg*3+2] += STEP_HEIGHT
    # print(traj_leg)
    traj_leg = gen_traj_all_legs(traj[leg_id].copy(), leg_id, get_X())
    LV, LO, LOmega = move_abs_one_leg(traj_leg, leg_id)
    step_len = len(LV)
    print(step_len)
    # R = reg_val * np.eye(18)
    # V = get_verins_12()
    # O = get_O()
    # omega = get_omega()
    # LV = [V]
    # LO = [O]
    # target = np.concatenate((traj_leg[0][0], traj_leg[1][0], traj_leg[2][0], traj_leg[3][0]))
    # for i in range(1, len(traj_leg[0])):
    #     # Computing dX
    #     X0 = direct_abs(V, O, omega)
    #     print("\nX0 =", X0)
    #     if distance(X0[leg_id * 3:leg_id * 3 + 3],
    #                 target[leg_id * 3:leg_id * 3 + 3]) > EPSILON:  # exit condition
    #         print("\nto far from original trajectory")
    #         print("distance = ", distance(X0[leg_id * 3:leg_id * 3 + 3],
    #               target[leg_id * 3:leg_id * 3 + 3]))
    #         break
    #     target = np.concatenate((traj_leg[0][i], traj_leg[1][i], traj_leg[2][i], traj_leg[3][i]))
    #     dX = target - X0
    #     print("target =", target)
    #     print("dX : ", dX)
    #     # Contraintes
    #     lb = np.full(18, - np.inf)
    #     ub = np.full(18, np.inf)
    #     for j in range(12):
    #         lb[j] = 450.0 - V[j]
    #         ub[j] = 650.0 - V[j]
    #     for j in range(3):
    #         if const_omega:
    #             lb[15 + j] = - max_omega * np.pi / 180 - omega[j]
    #             ub[15 + j] = max_omega * np.pi / 180 - omega[j]
    #         else:
    #             lb[15 + j] = - np.pi
    #             ub[15 + j] = np.pi
    #     # solve
    #     M = jacob_dX_to_dV_dO_dOmega(V, omega, direct_rel_12(V))
    #     P = M.T @ M + R
    #     q = - M.T @ dX
    #     sol = solve_qp(P, q, lb=lb, ub=ub)
    #     # Mise à jour des valeurs réelles
    #     V = V + sol[0:12]
    #     O = O + sol[12:15]
    #     omega = omega + sol[15:18]
    #     for v in V: assert 449.9 < v < 650.1, 'Elongation de vérin invalide'
    #     LV.append(V)
    #     LO.append(O)
    #     LOmega.append(omega)
    # # retour en position basse
    # step_len = len(LV)
    # print(step_len)
    # # print("\n0 : ", traj_leg[0])
    # # print("\n1 : ", traj_leg[1])
    # # print("\n2 : ", traj_leg[2])
    # # print("\n3 : ", traj_leg[3])
    # traj_leg = np.stack((traj_leg[0][0:step_len], traj_leg[1][0:step_len],
    #                            traj_leg[2][0:step_len], traj_leg[3][0:step_len]))
    # # print("\n0 : ", traj_leg[0])
    # # print("\n1 : ", traj_leg[1])
    # # print("\n2 : ", traj_leg[2])
    # # print("\n3 : ", traj_leg[3])
    # for leg in range(4):
    #     temp = traj_leg[leg][step_len].copy()
    #     for i in range(1, int(STEP_HEIGHT/20)):
    #         np.append(traj_leg[leg], temp.copy())
    #         print(i)
    #         traj_leg[leg][step_len+int(i/20)][2] -= i

    # for i in range(step_len + 1, len(traj_leg[0])):
    #     # Calcul de dX
    #     X0 = direct_abs(V, O, omega)
    #     target = np.concatenate((traj_leg[0][i], traj_leg[1][i], traj_leg[2][i], traj_leg[3][i]))
    #     if distance(X0[leg_id * 3:leg_id * 3 + 3],
    #                 target[i - 1][leg_id * 3:leg_id * 3 + 3]) > EPSILON:  # exit condition
    #         print("\n ALERTE, IMPOSSIBLE DE REDESCENDRE EN ETANT ASSEZ PRECIS !!! \n")
    #         assert(1 == 2)
    #     dX = target - X0
    #     # Contraintes
    #     lb = np.full(18, - np.inf)
    #     ub = np.full(18, np.inf)
    #     for j in range(12):
    #         lb[j] = 450.0 - V[j]
    #         ub[j] = 650.0 - V[j]
    #     for j in range(3):
    #         if const_omega:
    #             lb[15 + j] = - max_omega * np.pi / 180 - omega[j]
    #             ub[15 + j] = max_omega * np.pi / 180 - omega[j]
    #         else:
    #             lb[15 + j] = - np.pi
    #             ub[15 + j] = np.pi
    #     # solve
    #     M = jacob_dX_to_dV_dO_dOmega(V, omega, direct_rel_12(V))
    #     P = M.T @ M + R
    #     q = - M.T @ dX
    #     sol = solve_qp(P, q, lb=lb, ub=ub)
    #     # Mise à jour des valeurs réelles
    #     V = V + sol[0:12]
    #     O = O + sol[12:15]
    #     omega = omega + sol[15:18]
    #     for v in V: assert 449.9 < v < 650.1, 'Elongation de vérin invalide'
    #     LV.append(V)
    #     LO.append(O)
    #     LOmega.append(omega)
    return LV, LO, LOmega, traj_leg


################################ COM CONTROL ##################################


def gen_G(l1, l2, l3, V, Omega, com, passenger=True, passenger_weight=80):
    """
    Calcule la matrice G associée à la contrainte G @ x < h dans le solveur de move_abs_all_legs() permettant
    de garantir le maintien du centre du masse du robot au dessus du polygone de sustentation formé par ses pattes

    :param l1: coordonnées de la 1ère patte au sol
    :param l2: coordonnées de la 2nde patte au sol
    :param l3: coordonnées de la 3ème patte au sol
    :param V: élongations des vérins à l'itération précédente
    :param Omega: valeurs de Omega à l'itération précédente
    :param com: coordonnées du centre de masse à l'itération précédente
    :return: G
    """
    M = [np.array([[0 if (l2 - l1)[1] == 0.0 else (l2 - l1)[1] / abs((l2 - l1)[1]), 0],
                   [0, 0 if (l2 - l1)[0] == 0.0 else - (l2 - l1)[0] / abs((l2 - l1)[0])]]),
         np.array([[0 if (l3 - l2)[1] == 0.0 else (l3 - l2)[1] / abs((l3 - l2)[1]), 0],
                   [0, 0 if (l3 - l2)[0] == 0.0 else - (l3 - l2)[0] / abs((l3 - l2)[0])]]),
         np.array([[0 if (l1 - l3)[1] == 0.0 else (l1 - l3)[1] / abs((l1 - l3)[1]), 0],
                   [0, 0 if (l1 - l3)[0] == 0.0 else - (l1 - l3)[0] / abs((l1 - l3)[0])]])]
    J_com = gen_J_com_abs(V, Omega, com, passenger_weight=passenger_weight)[0: 2, 0: 18] if passenger \
        else gen_J_com_abs(V, Omega, com, passenger_weight=0)[0: 2, 0: 18]
    return np.concatenate((M[0] @ J_com, M[1] @ J_com, M[2] @ J_com))


def gen_h(l1, l2, l3, com):
    """
    Calcule la matrice h associée à la contrainte G @ x < h dans le solveur de move_abs_all_legs() permettant
    de garantir le maintien du centre du masse du robot au dessus du polygone de sustentation formé par ses pattes

    :param l1: coordonnées de la 1ère patte au sol
    :param l2: coordonnées de la 2nde patte au sol
    :param l3: coordonnées de la 3ème patte au sol
    :param com: coordonnées du centre de masse à l'itération précédente
    :return: h
    """
    return np.array([1 if (l2 - l1)[1] == 0.0 else (l2 - l1)[1] / abs((l2 - l1)[1]) *
                                                   (((l2 - l1)[0] * (com - l1)[1] / (l2 - l1)[1]) + l1[0]),
                     1 if (l2 - l1)[0] == 0.0 else - (l2 - l1)[0] / abs((l2 - l1)[0]) *
                                                   (((l2 - l1)[1] * (com - l1)[0] / (l2 - l1)[0]) + l1[1]),
                     1 if (l3 - l2)[1] == 0.0 else (l3 - l2)[1] / abs((l3 - l2)[1]) *
                                                   (((l3 - l2)[0] * (com - l2)[1] / (l3 - l2)[1]) + l2[0]),
                     1 if (l3 - l2)[0] == 0.0 else - (l3 - l2)[0] / abs((l3 - l2)[0]) *
                                                   (((l3 - l2)[1] * (com - l2)[0] / (l3 - l2)[0]) + l2[1]),
                     1 if (l1 - l3)[1] == 0.0 else (l1 - l3)[1] / abs((l1 - l3)[1]) *
                                                   (((l1 - l3)[0] * (com - l3)[1] / (l1 - l3)[1]) + l3[0]),
                     1 if (l1 - l3)[0] == 0.0 else - (l1 - l3)[0] / abs((l1 - l3)[0]) *
                                                   (((l1 - l3)[1] * (com - l3)[0] / (l1 - l3)[0]) + l3[1])])


def gen_traj_move_abs(traj_leg, leg_id, traj_com, X0):
    traj_all_legs = gen_traj_all_legs(traj_leg, leg_id, X0)
    return [[l[0], l[1], l[2], l[3], l[4], l[5], l[6], l[7], l[8], l[9], l[10], l[11], com[0], com[1], com[2]]
            for (l, com) in zip(traj_all_legs, traj_com)]


def gen_vertices_poly(l1, l2, l3, offset=0.5):
    """
    Calcule les sommets du triangle dans lequel le projeté du centre de masse du robot doit être maintenu.
    Ce triangle correspond au triangle formé par les extrémités des pattes au sol du robot auquel on soustrait
    une partie de la surface à proximité de ses bords paar sécurité

    :param l1: coordonnées de la 1ère patte au sol
    :param l2: coordonnées de la 2nde patte au sol
    :param l3: coordonnées de la 3ème patte au sol
    :param offset: flottant compris entre 0 et 1 traduisant l'intensité de la réduction du triangle de sustentation
    :return: liste des sommets du triangle de sustentation autorisé
    """
    M = [(l2 + l3) / 2, (l1 + l3) / 2, (l1 + l2) / 2]
    return [offset * M[0] + (1 - offset) * l1, offset * M[1] + (1 - offset) * l2, offset * M[2] + (1 - offset) * l3]


def jacob_dX_dcom_to_dV_dO_dOmega(V, Omega, X_rel, com, passenger_weight=80):
    return np.concatenate((jacob_dX_to_dV_dO_dOmega(V, Omega, X_rel),
                           gen_J_com_abs(V, Omega, com, passenger_weight=passenger_weight)))


def move_abs_with_com_constraint(traj_leg, leg_id, reg_val=0.01, const_omega=True, max_omega=10,
                                 passenger_weight=80., offset=0):
    """
    Détermine la liste des élongations des vérins, des positions du centre O du robot et des valeurs de Omega
    successives permettant à l'extrémité de la patte leg_id de suivre la trajectoire traj_leg (en coordonnées absolues)
    tout en contraignant le centre de masse à se maintenir dans le triangle de sustentation formé par les pattes
    au sol du robot

    dX, dcom -> dV, dO, dOmega

    :param traj_leg: trajectoire de l'extrémités de la patte leg_id en coordonnées absolues
    :param leg_id: ID de la patte en l'air
    :param reg_val: coefficient de la régularisation dans la minimisation de l'erreur quadratique de position
    :param const_omega: booléen activant ou non la contrainte sur Omega
    :param max_omega: angles maximaux permis au châssis
    :param passenger_weight: poids du passager s'il y en a un
    :param offset: offset de sécurité sur le triangle de sustentation
    :return: valeurs successives de (V, O, Oméga) au cours du déplacement
    """
    V = get_verins_12()
    O = get_O()
    Omega = get_omega()

    LV = [V]
    LO = [O]
    LOmega = [Omega]

    # Construction de la trajectoire pour toutes les pattes
    traj = gen_traj_all_legs(traj_leg, leg_id, direct_abs(V, O, Omega))

    # Régularisation
    R = reg_val * np.eye(18)

    # Triangle de sustentation
    legs_on_ground = []
    for j in range(4):
        if j != leg_id:
            legs_on_ground.append(get_leg_pos(j))
    T = gen_vertices_poly(legs_on_ground[0], legs_on_ground[1], legs_on_ground[2], offset=offset)

    for i in range(1, len(traj)):
        # Calcul de dX
        X0 = direct_abs(V, O, Omega)
        set_X(X0)
        dX = traj[i] - X0

        # Contraintes lb <= x <= ub (élongations des vérins et angle max du châssis)
        lb = np.full(18, - np.inf)
        ub = np.full(18, np.inf)
        for j in range(12):
            lb[j] = 450.0 - V[j]
            ub[j] = 650.0 - V[j]
        for j in range(3):
            if const_omega:
                lb[15 + j] = - max_omega * np.pi / 180 - Omega[j]
                ub[15 + j] = max_omega * np.pi / 180 - Omega[j]
            else:
                lb[15 + j] = - np.pi
                ub[15 + j] = np.pi

        # Contraintes G @ x <= h (centre de masse)
        com = robot_ref_to_abs(center_of_mass(V, passenger_weight=passenger_weight), O, Omega)
        set_com(com)
        G = gen_G(T[0], T[1], T[2], V, Omega, com, passenger_weight=passenger_weight)
        h = gen_h(T[0], T[1], T[2], com)

        # Application du solveur
        M = jacob_dX_to_dV_dO_dOmega(V, Omega, direct_rel_12(V))
        P = M.T @ M + R
        q = - M.T @ dX
        sol = solve_qp(P, q, lb=lb, ub=ub, G=G, h=h)
        V = V + sol[0:12]
        O = O + sol[12:15]
        Omega = Omega + sol[15:18]

        # Mise à jour des valeurs réelles
        set_verins_12(V)
        set_O(O)
        set_omega(Omega)
        for v in V: assert 449.9 < v < 650.1, 'Elongation de vérin invalide'
        LV.append(V)
        LO.append(O)
        LOmega.append(Omega)
    return LV, LO, LOmega


def move_abs(traj_leg, leg_id, traj_com, reg_val=0.01, const_omega=True, max_omega=10, passenger_weight=80.):
    """
    Détermine la liste des élongations des vérins, des positions du centre O du robot et des valeurs de Omega
    successives permettant à l'extrémité de la patte leg_id de suivre la trajectoire traj_leg et au centre de masse
    de suivre traj_com (avec des trajectoires en coordonnées absolues

    dX, dcom -> dV, dO, dOmega

    :param traj_leg: trajectoire de l'extrémités de la patte leg_id en coordonnées absolues
    :param leg_id: ID de la patte en l'air
    :param traj_com: trajectoire du centre de masse du robot en absolu
    :param reg_val: coefficient de la régularisation dans la minimisation de l'erreur quadratique de position
    :param const_omega: booléen activant ou non la contrainte sur Omega
    :param max_omega: angles maximaux permis au châssis
    :param passenger_weight: poids du passager s'il y en a un, 0 sinon
    :return: valeurs successives de (V, O, Oméga) au cours du déplacement
    """
    V = get_verins_12()
    O = get_O()
    Omega = get_omega()

    LV = [V]
    LO = [O]
    LOmega = [Omega]

    # Construction de la trajectoire pour toutes les pattes et le centre de masse du robot
    traj = gen_traj_move_abs(traj_leg, leg_id, traj_com, direct_abs(V, O, Omega))

    # Régularisation
    R = reg_val * np.eye(18)

    for i in range(1, len(traj)):
        # Calcul de dX
        X0 = direct_abs(V, O, Omega)
        set_X(X0)
        com = robot_ref_to_abs(center_of_mass(V, passenger_weight=passenger_weight), O, Omega)
        set_com(com)
        dX = traj[i] - np.concatenate((X0, com))

        # Contraintes lb <= x <= ub (élongations des vérins et angle max du châssis)
        lb = np.full(18, - np.inf)
        ub = np.full(18, np.inf)
        for j in range(12):
            lb[j] = 450.0 - V[j]
            ub[j] = 650.0 - V[j]
        for j in range(3):
            if const_omega:
                lb[15 + j] = - max_omega * np.pi / 180 - Omega[j]
                ub[15 + j] = max_omega * np.pi / 180 - Omega[j]
            else:
                lb[15 + j] = - np.pi
                ub[15 + j] = np.pi

        # Application du solveur
        M = jacob_dX_dcom_to_dV_dO_dOmega(V, Omega, direct_rel_12(V), com, passenger_weight=passenger_weight)
        P = M.T @ M + R
        q = - M.T @ dX
        sol = solve_qp(P, q, lb=lb, ub=ub)
        V = V + sol[0:12]
        O = O + sol[12:15]
        Omega = Omega + sol[15:18]

        # Mise à jour des valeurs réelles
        set_verins_12(V)
        set_O(O)
        set_omega(Omega)
        for v in V: assert 449.9 < v < 650.1, 'Elongation de vérin invalide'
        LV.append(V)
        LO.append(O)
        LOmega.append(Omega)
    return LV, LO, LOmega


def planning(target, com):
    return 0


################################ SIMPLE WALK ##################################

def init_com(radius=150, passenger_weight=80.0, nb_steps=20):
    P = get_leg_pos(0)
    L = np.linspace(0, radius * np.sqrt(2) / 2, nb_steps)
    traj_leg = np.full((nb_steps, 3), [P[0], P[1], P[2]])
    traj_com = [[-l, -l] for l in L]

    return move_abs_legs_and_com(traj_leg, 0, traj_com, passenger_weight=passenger_weight)


def gen_com_traj(leg_id, nb_steps, radius=150):
    """
    Génère la prochaine position de com durant le déplacement de la jambe leg_id afin qu'il suive une trajectoire
    circulaire autour de l'intersection des diagonales du quadrilatère formé par le projeté des pattes au sol.

    :param leg_id: patte en mouvement (en l'air)
    :param nb_steps: nombre de point restant dans la trajectoire de leg_id
    :param radius: rayon de la trajectoire circulaire du com
    :return: prochain position du com
    """
    center = intersection_legs()
    com = get_com()[0:2] - center

    endpoint = get_endpoint(leg_id, center, r=radius)
    d_com_center = distance(com)
    calpha = min(((com[0] - center[0]) * endpoint[0] + (com[1] - center[1]) * endpoint[1]) / (d_com_center * radius), 1)
    # print(calpha)
    alpha = np.arccos(calpha)
    beta = - (nb_steps - 1) * alpha / nb_steps
    R = np.array([[np.cos(beta), -np.sin(beta)],
                  [np.sin(beta), np.cos(beta)]])
    return center + R @ endpoint


def move_abs_leg_autocom(traj, leg_id, com_radius=150, reg_val=0.01, const_omega=True, max_omega=10, passenger_weight=80.0):
    """
    Fait suivre la trajectoire traj à leg_id en faisant effectuer sa rotation autour du centre à com.
    Suppose que com soit à l'opposée de leg_id sur son cercle de parcours au début du mouvement.
    """
    V = get_verins_12()
    O = get_O()
    Omega = get_omega()
    LV = [V]
    LO = [O]
    LOmega = [Omega]

    X0 = direct_abs(V, O, Omega)
    traj_legs = gen_traj_all_legs(traj, leg_id, X0)

    # Régularisation
    R = reg_val * np.eye(18)

    nb_steps = len(traj_legs)
    for i in range(1, nb_steps):
        # Calcul de dX
        X0 = direct_abs(V, O, Omega)
        set_X(X0)
        com = robot_ref_to_abs(center_of_mass(V, passenger_weight=passenger_weight), O, Omega)
        set_com(com)
        dX = np.concatenate((traj_legs[i] - X0, gen_com_traj(leg_id, nb_steps - i, radius=com_radius) - com[0:2]))

        # Contraintes
        lb = np.full(18, - np.inf)
        ub = np.full(18, np.inf)
        for j in range(12):
            lb[j] = 450.0 - V[j]
            ub[j] = 650.0 - V[j]
        for j in range(3):
            if const_omega:
                lb[15 + j] = - max_omega * np.pi / 180 - Omega[j]
                ub[15 + j] = max_omega * np.pi / 180 - Omega[j]
            else:
                lb[15 + j] = - np.pi
                ub[15 + j] = np.pi

        # Application du solveur
        M_X = jacob_dX_to_dV_dO_dOmega(V, Omega, direct_rel_12(V))
        M_COM = gen_J_com_abs(V, Omega, com, passenger_weight=passenger_weight)[0: 2, 0: 18]
        M = np.concatenate((M_X, M_COM))
        P = M.T @ M + R
        q = - M.T @ dX
        sol = solve_qp(P, q, lb=lb, ub=ub)
        V = V + sol[0:12]
        O = O + sol[12:15]
        Omega = Omega + sol[15:18]

        # Mise à jour des valeurs réelles
        set_verins_12(V)
        set_O(O)
        set_omega(Omega)
        for v in V: assert 449.9 < v < 650.1, 'Elongation de vérin invalide'
        LV.append(V)
        LO.append(O)
        LOmega.append(Omega)
    X0 = direct_abs(V, O, Omega)
    set_X(X0)
    com = robot_ref_to_abs(center_of_mass(V, passenger_weight=passenger_weight), O, Omega)
    set_com(com)
    return LV, LO, LOmega


def legs_up(nb_steps=30, amp=200, com_radius=150, passenger_weight=80.0):
    LV = init_com(nb_steps=20)[0]

    traj_leg = traj_abs_sin_1(nb_steps, amp, 0)
    lv = move_abs_leg_autocom(traj_leg, 0, com_radius=com_radius, passenger_weight=passenger_weight)[0]
    for i in range(nb_steps):
        LV.append(lv[i])

    traj_leg = traj_abs_sin_1(nb_steps, amp, 2)
    lv = move_abs_leg_autocom(traj_leg, 2, com_radius=com_radius, passenger_weight=passenger_weight)[0]
    for i in range(nb_steps):
        LV.append(lv[i])

    traj_leg = traj_abs_sin_1(nb_steps, amp, 3)
    lv = move_abs_leg_autocom(traj_leg, 3, com_radius=com_radius, passenger_weight=passenger_weight)[0]
    for i in range(nb_steps):
        LV.append(lv[i])

    traj_leg = traj_abs_sin_1(nb_steps, amp, 1)
    lv = move_abs_leg_autocom(traj_leg, 1, com_radius=com_radius, passenger_weight=passenger_weight)[0]
    for i in range(nb_steps):
        LV.append(lv[i])

    return LV

################################ DEPRECATED ###################################


def compute_traj_from_joystick_abs_equals_nb_points(joystick):
    """
    Compute all 4 trajectories from joystick input with the same number of point for each discretization

    :param joystick: ((rota_center_x, rota_center_y), (dir_x, dir_y))
    :return: traj[step_of_traj][12_coords_(->_4_points)]
    """
    centre, direction = joystick[0], joystick[1]
    direction = unitary_vec(direction)
    traj = []
    r = []
    pos = direct_abs(get_verins_12(), get_O(), get_omega())
    # Computing radii
    for leg in range(4):
        radius = distance(pos[leg * 3 + 0:leg * 3 + 3], (centre[0], centre[1], 0))
        r.append(radius)
    r_max = max(r)
    n = int(2 * np.pi * r_max / TRAJ_ACCUR)
    for i in range(n - 100):
        L = []
        for leg in range(4):
            alpha = np.arccos(abs((centre[1] - pos[3 * leg + 1])) / r[leg])
            signe_cx = (centre[0] - pos[leg * 3 + 0]) / abs(centre[0] - pos[leg * 3 + 0])
            signe_cy = (centre[1] - pos[leg * 3 + 1]) / abs(centre[1] - pos[leg * 3 + 1])
            if signe_cx < 0 and signe_cy < 0:
                alpha = + np.pi / 2 - alpha
            if signe_cx > 0 > signe_cy:
                alpha = + np.pi / 2 + alpha
            if signe_cx < 0 < signe_cy:
                alpha = - np.pi / 2 + alpha
            if signe_cx > 0 and signe_cy > 0:
                alpha = - np.pi / 2 - alpha
            L = np.append(L, (r[leg] * np.cos((2 * i * np.pi) / n + alpha) + centre[0],
                              r[leg] * np.sin((2 * i * np.pi) / n + alpha) + centre[1],
                              pos[3 * leg + 2]))
        traj.append(L)
    return traj


def furthest_accessible_abs(traj, leg_id, max_omega=10, const_omega=True, reg_val=0.01, step_height=0):
    """
    Compute the furthest point reachable of the trajectory for one leg
    traj should be the trajectory of all legs (basically returned by compute_traj_from_joystick)
    :param traj: trajectory to follow
    :param leg_id: ID of the leg
    :param max_omega: maximum angle allowed for the body
    :param const_omega: bool -> is there a constraint on angle
    :param reg_val: regularisation value
    :param step_height: height of one step
    :return: the furthest point accessible from the traj
    """
    temp = np.zeros(12)
    traj_leg = [temp] * len(traj)
    # traj_leg = np.zeros((len(traj), 12))
    for i in range(len(traj)):
        traj_leg[i] = traj[0].copy()
        traj_leg[i][leg_id * 3:leg_id * 3 + 3] = traj[i][leg_id * 3:leg_id * 3 + 3]
    init = traj[0].copy()
    for j in range(step_height):
        traj_leg.insert(0, init.copy())
        traj_leg[0][leg_id * 3 + 2] += step_height + 1 - j
    for k in range(step_height, len(traj_leg)):
        traj_leg[k][leg_id * 3 + 2] += step_height
    R = reg_val * np.eye(18)
    V = get_verins_12()
    O = get_O()
    omega = get_omega()
    LV = [V]
    LO = [O]
    LOmega = [omega]
    for i in range(1, len(traj_leg)):
        # Calcul de dX
        X0 = direct_abs(V, O, omega)
        if distance(X0[leg_id * 3:leg_id * 3 + 3], traj_leg[i - 1][leg_id * 3:leg_id * 3 + 3]) > 20:  # exit condition
            return i - step_height, (LV, LO, LOmega)
        set_X(X0)
        dX = traj_leg[i] - X0
        # Contraintes
        lb = np.full(18, - np.inf)
        ub = np.full(18, np.inf)
        for j in range(12):
            lb[j] = 450.0 - V[j]
            ub[j] = 650.0 - V[j]
        for j in range(3):
            if const_omega:
                lb[15 + j] = - max_omega * np.pi / 180 - omega[j]
                ub[15 + j] = max_omega * np.pi / 180 - omega[j]
            else:
                lb[15 + j] = - np.pi
                ub[15 + j] = np.pi
        # solve
        M = jacob_dX_to_dV_dO_dOmega(V, omega, direct_rel_12(V))
        P = M.T @ M + R
        q = - M.T @ dX
        sol = solve_qp(P, q, lb=lb, ub=ub)
        # Mise à jour des valeurs réelles
        V = V + sol[0:12]
        O = O + sol[12:15]
        omega = omega + sol[15:18]
        for v in V: assert 449.9 < v < 650.1, 'Elongation de vérin invalide'
        LV.append(V)
        LO.append(O)
        LOmega.append(omega)
    return len(traj_leg) - 1, (LV, LO, LOmega)


def furthest_accessible_all_legs_abs(traj, step_height=0):
    """

    :param traj:
    :param step_height:
    :return:
    """
    max_step, data = [], []
    for leg in range(4):
        step, data_leg = furthest_accessible_abs(traj, leg, step_height=step_height, max_omega=45)
        max_step.append(step)
        data.append((data_leg))
    return max_step, data


############################################################################

if __name__ == "__main__":
    import doctest

    doctest.testmod()
