from upg_kinematic import *


def move_abs_one_leg(traj, leg_id, reg_val=0.01, const_omega=True, max_omega=10):
    """
    Détermine la liste des élongations successives des vérins permettant à l'extrémité de la patte leg_id, au centre
    du robot et à l'angle du châssis de suivre la trajectoire traj données

    dP(OnAir), dO, dOmega -> dP(OnGround), dV, dO, dOmega

    :param traj: trajectoire de l'extrémité de la patte, du centre du robot et évolution de l'angle du châssis
    :param leg_id: patte dans les airs (qui bouge selon traj)
    :param reg_val: coefficient de la régularisation dans la minimisation de l'erreur quadratique de position
    :param max_omega: angles maximaux permis au châssis (en m et n)
    :return: valeurs successives de (V, O, Oméga) au cours du déplacement
    """
    V = get_verins_12()
    O = get_O()
    Omega = get_omega()
    X_ini = get_X()

    LV = [V]
    LO = [O]
    LOmega = [Omega]

    # Régularisation des dV (et plus)
    # R = np.concatenate((np.concatenate((reg_val * np.eye(12), np.zeros((12, 15))), axis=1), np.zeros((15, 27))))
    R = reg_val * np.eye(27)
    # R = np.concatenate((np.concatenate((reg_val * np.eye(21), np.zeros((21, 6))), axis=1), np.zeros((6, 27))))

    for i in range(1, len(traj)):
        # Calcul de dX
        X0 = direct_abs(V, O, Omega)
        set_X(X0)
        dX = traj[i] - np.append(np.append(X0[leg_id * 3: leg_id * 3 + 3], O), Omega)
        print("traj : ", traj[i])
        print("X0 : ", np.append(np.append(X0[leg_id * 3: leg_id * 3 + 3], O), Omega))
        print("dX : ", dX)

        # Contraintes (soft)
        lb = np.full(27, - np.inf)
        ub = np.full(27, np.inf)
        for j in range(12):
            lb[j] = 450.0 - V[j]
            ub[j] = 650.0 - V[j]
        for j in range(3):
            if const_omega:
                lb[24 + j] = - max_omega * np.pi / 180 - Omega[j]
                ub[24 + j] = max_omega * np.pi / 180 - Omega[j]
            else:
                lb[24 + j] = - np.pi
                ub[24 + j] = np.pi

        # Contraintes (hard)
        A = np.concatenate((np.zeros((12, 27)),
                            np.concatenate((np.zeros((9, 12)), np.eye(9), np.zeros((9, 6))), axis=1),
                            np.zeros((6, 27))))
        b = np.zeros(27)
        correction_X = X_ini - get_X()
        print(X_ini)
        k = 0
        for j in range(4):
            if j != leg_id:
                b[12 + k * 3] = correction_X[j * 3]
                b[13 + k * 3] = correction_X[j * 3 + 1]
                b[14 + k * 3] = correction_X[j * 3 + 2]
                k += 1

        # Application du solveur
        M = jacob_dPf_dO_dOmega_to_dV_dPg_dO_dOmega(leg_id, V, Omega, direct_rel_12(V))
        P = M.T @ M + R
        q = np.reshape(- dX @ M, 27)
        sol = solve_qp(P, q, lb=lb, ub=ub, A=A, b=b)
        V = V + sol[0:12]
        O = O + sol[21:24]
        Omega = Omega + sol[24:27]

        # Mise à jour des valeurs réelles
        set_verins_12(V)
        set_O(O)
        set_omega(Omega)
        for v in V: assert 449.9 < v < 650.1, 'Elongation de vérin invalide'
        LV.append(V)
        LO.append(O)
        LOmega.append(Omega)
    return LV, LO, LOmega
