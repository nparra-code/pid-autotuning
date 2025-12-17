from functions import *

import control as ct

n = 20 # number of random sets
# ---- Generate n independent sets ----
random_param_sets = [generate_random_motor_params() for _ in range(n)]

print("----------- Generated parameter sets: ----------------\n")
for i, p in enumerate(random_param_sets):
    print(f"Set #{i+1}: {p}\n")
print("------------------------------------------------------")


# ---- Define trajectories ----------
mov_touples_list_list = [
    [
        ("linear", True,  1,    0, 0, 3),
        ("linear", True,  1, -135, 0, 5),
        ("linear", False, 1,  -90, 0, 4)
    ],
    [
        ("linear", True,  2,   0, 0, 4),
        ("linear", True,  2, -90, 0, 4),
        ("linear", False, 2,   0, 0, 4),
        ("linear", False, 2, -90, 0, 4)

    ]

]

Ts = 0.01
for i, motor_params in enumerate(random_param_sets):

    print(f"\n===== Running robot set #{i+1} =====")

    # Update system model A, B, C, D for this motor param set
    sys_lin = ct.nlsys(
        updfnc_omniwheel, outfnc_omniwheel, params=motor_params,
        inputs=('u1', 'u2', 'u3'),  outputs=('x', 'y', 'phi', 'dx', 'dy', 'dphi'),
        name='omniwheel_sys', states=('x', 'y', 'phi', 'dx', 'dy', 'dphi')
    )

    Kp = np.array([21.1, 20.4, 21.1])
    Ki = np.array([18.1, 15.9, 18.1])
    Kd = np.array([0.03, 0.02, 0.03])

    # Find the equilibrium point
    X0 = [0, 0, 0, 0, 0, 0]
    eqpt = ct.find_eqpt(sys_lin, X0, 0)
    xeq = eqpt[0]

    # Linearize the system at the equilibrium point
    lin_omni = ct.linearize(sys_lin, xeq, 0)
    sys_z = ct.c2d(lin_omni, Ts, method='zoh')  # Convert to discrete-time system
    A_z, B_z, C_z, D_z = sys_z.A, sys_z.B, sys_z.C, sys_z.D

    sys_lin_z = ct.StateSpace(A_z, B_z, C_z, D_z, Ts)

    for mov_tuples_list in mov_touples_list_list:
        omega_ref, T = generate_omega_ref_trajectory(
            mov_tuples_list=mov_tuples_list,
            Ts = Ts
        )

        run_pid_sim(Kp, Ki, Kd, A_z, B_z, C_z, D_z, T, omega_ref,
                    save_to_file=True, file_name=f"Robot{i+1}",
                    directory="/content/drive/MyDrive/pid_autotuning/data/robots_train/")