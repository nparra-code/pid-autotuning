
from functions import *

import control as ct
from itertools import product

Ts = 0.01
time = 10
T = np.arange(0, time, Ts)

# ---- Generate 4 independent sets ----
random_param_sets = [generate_random_motor_params() for _ in range(4)]

print("Generated parameter sets:\n")
for i, p in enumerate(random_param_sets):
    print(f"Set #{i+1}: {p}\n")

print("------------------------------------------------------")

all_runs_results = []

omega_ref, T = generate_omega_ref_trajectory(
    mov_tuples_list=[
        ('linear', True,  1,   0, 0, 2),
        ('linear', True,  1, -90, 0, 2),
        ('linear', False, 1,   0, 0, 2),
        ('linear', False, 1, -90, 0, 2)
    ], Ts=Ts
)

for i, motor_params in enumerate(random_param_sets):

    print(f"\n===== Running PID exploration for parameter set #{i+1} =====")

    # Update system model A, B, C, D for this motor param set
    sys_lin = ct.nlsys(
        updfnc_omniwheel, outfnc_omniwheel, params=motor_params,
        inputs=('u1', 'u2', 'u3'),  outputs=('x', 'y', 'phi', 'dx', 'dy', 'dphi'),
        name='omniwheel_sys', states=('x', 'y', 'phi', 'dx', 'dy', 'dphi')
    )

    # Find the equilibrium point
    X0 = [0, 0, 0, 0, 0, 0]
    eqpt = ct.find_eqpt(sys_lin, X0, 0)
    xeq = eqpt[0]

    # Linearize the system at the equilibrium point
    lin_omni = ct.linearize(sys_lin, xeq, 0)
    sys_z = ct.c2d(lin_omni, Ts, method='zoh')  # Convert to discrete-time system
    A_z, B_z, C_z, D_z = sys_z.A, sys_z.B, sys_z.C, sys_z.D

    sys_lin_z = ct.StateSpace(A_z, B_z, C_z, D_z, Ts)

    # --- Parameter grids ---
    Kp_vals1 = np.arange(20.9, 21.1, 0.1)
    Ki_vals1 = np.arange(17.9, 18.1, 0.1)
    Kd_vals1 = np.arange(0.02, 0.04, 0.01)

    Kp_vals2 = np.arange(20.4, 21.6, 0.1)
    Ki_vals2 = np.arange(15.9, 16.1, 0.1)
    Kd_vals2 = Kd_vals1

    Kp_vals3 = Kp_vals1
    Ki_vals3 = Ki_vals1
    Kd_vals3 = Kd_vals1

    # Now reuse your PID grid-search exactly as before:
    results = []
    i = 0

    for Kp_val1, Ki_val1, Kd_val1 in product(Kp_vals1, Ki_vals1, Kd_vals1):
        for Kp_val2, Ki_val2, Kd_val2 in product(Kp_vals2, Ki_vals2, Kd_vals2):
            for Kp_val3, Ki_val3, Kd_val3 in product(Kp_vals3, Ki_vals3, Kd_vals3):

                Kp = np.array([Kp_val1, Kp_val2, Kp_val3])
                Ki = np.array([Ki_val1, Ki_val2, Ki_val3])
                Kd = np.array([Kd_val1, Kd_val2, Kd_val3])

                set_err, overshoot, osc, y_log, U_pid = \
                    run_pid_sim(Kp, Ki, Kd, A_z, B_z, C_z, D_z, T, omega_ref, save_to_file=False, file_name="", directory="")

                results.append({
                    "iteration": i + 1,
                    "Kp": Kp, "Ki": Ki, "Kd": Kd,
                    "SetErr": set_err, "Overshoot": overshoot, "Oscillation": osc
                })

                if i == 100:
                  print(f"Kp1={Kp_val1:.1f}, Ki1={Ki_val1:.1f}, Kd1={Kd_val1:.2f} | "
                        f"Kp2={Kp_val2:.1f}, Ki2={Ki_val2:.1f}, Kd2={Kd_val2:.2f} | "
                        f"Kp3={Kp_val3:.1f}, Ki3={Ki_val3:.1f}, Kd3={Kd_val3:.2f} | "

                        f"Err={set_err:.3f}, Ov={overshoot:.3f}, Osc={osc:.4f}")
                  i = -1
                i+=1

    all_runs_results.append(results)

import pandas as pd
flat_results = [item for sublist in all_runs_results for item in sublist]
df = pd.DataFrame(flat_results)

# Sort by lowest steady-state error, then overshoot
df['score'] = df['SetErr']*3 + df['Overshoot']*2 + df['Oscillation']*0.5
df_sorted = df.sort_values(by='score')

print("\n=== Top 5 PID configurations ===")
print(df_sorted.head())