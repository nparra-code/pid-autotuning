def s_z_domain(Kp_cont, Ti_cont, Td_cont, Ts):
    # 1. Proportional Term (remains the same)
    Kp_disc = Kp_cont

    # 2. Integral Term (K_I = K_P / T_I)
    # The integral gain in the discrete positional form (Ki_z) is K_P * (Ts / T_I)
    if Ti_cont == 0:
        Ki_z = 0.0 # No integral action
    else:
        if type(Kp_cont) == float:
            Ki_z = Kp_cont * (Ts / 2)
        else:
            Ki_z = (Kp_cont[0]+Kp_cont[1]) * (Ts / 2) / 2

    # 3. Derivative Term (K_D = K_P * T_D)
    # The derivative gain constant (Kd_z) is K_P * (T_D / Ts)
    Kd_z = (Td_cont / Ts)

    return {
        "Kp_disc": Kp_disc,
        "Ki_z": Ki_z,
        "Kd_z": Kd_z
    }

# s_z_domain(Kp_cont=51, Ti_cont=0.02, Td_cont=0.003, Ts=0.001)